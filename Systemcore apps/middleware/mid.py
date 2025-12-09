"""
CAN Bus Middleware with Auto-Discovery and Topic Management
Requires: python-can library (pip install python-can)
"""

import can
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Any, Callable
from enum import IntEnum
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DataType(IntEnum):
    """Supported data types"""
    INT8 = 0
    INT32 = 1
    FLOAT = 2
    BOOL = 3
    ASCII_STRING = 4


class MessageType(IntEnum):
    """CAN message types for protocol"""
    DISCOVERY = 0x00      # Device announces presence
    CLAIM_PUB = 0x01      # Claim topic for publishing
    CLAIM_SUB = 0x02      # Claim topic for subscribing
    DATA = 0x03           # Data transmission
    HEARTBEAT = 0x04      # Keep-alive message
    ERROR = 0x05          # Error/conflict notification


@dataclass
class TopicInfo:
    """Information about a topic"""
    topic_id: int
    data_type: DataType
    publisher_device: Optional[int] = None
    last_value: Any = None
    last_timestamp: float = 0.0
    subscribers: set = field(default_factory=set)


@dataclass
class DeviceInfo:
    """Information about a device on the network"""
    device_id: int
    last_seen: float = field(default_factory=time.time)
    published_topics: set = field(default_factory=set)
    subscribed_topics: set = field(default_factory=set)


class CANBusManager:
    """Manages a single CAN bus interface"""
    
    def __init__(self, bus_id: int, device_id: int, channel: str = 'can0', bitrate: int = 500000):
        self.bus_id = bus_id
        self.device_id = device_id
        self.channel = channel
        self.bitrate = bitrate
        
        # Internal state
        self.topics: Dict[int, TopicInfo] = {}
        self.devices: Dict[int, DeviceInfo] = {}
        self.running = False
        self.bus: Optional[can.Bus] = None
        
        # Locks for thread safety
        self.lock = threading.RLock()
        
        # Callbacks for value updates
        self.callbacks: Dict[int, list] = {}
        
        # Threads
        self.recv_thread: Optional[threading.Thread] = None
        self.heartbeat_thread: Optional[threading.Thread] = None
        
    def start(self):
        """Initialize and start the CAN bus"""
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype='socketcan',
                bitrate=self.bitrate
            )
            self.running = True
            
            # Register our device
            with self.lock:
                self.devices[self.device_id] = DeviceInfo(device_id=self.device_id)
            
            # Start threads
            self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.recv_thread.start()
            
            self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self.heartbeat_thread.start()
            
            # Send discovery message
            self._send_discovery()
            
            logger.info(f"CAN bus {self.bus_id} started on {self.channel} (device {self.device_id})")
            
        except Exception as e:
            logger.error(f"Failed to start CAN bus: {e}")
            raise
    
    def stop(self):
        """Stop the CAN bus"""
        self.running = False
        if self.recv_thread:
            self.recv_thread.join(timeout=1.0)
        if self.heartbeat_thread:
            self.heartbeat_thread.join(timeout=1.0)
        if self.bus:
            self.bus.shutdown()
        logger.info(f"CAN bus {self.bus_id} stopped")
    
    def _build_arbitration_id(self, msg_type: MessageType, device_id: int, topic_id: int = 0) -> int:
        """
        Build CAN arbitration ID (11-bit standard)
        Format: [msg_type:3][device_id:8]
        For extended (29-bit): [msg_type:3][device_id:8][topic_id:16][reserved:2]
        """
        # Using 29-bit extended format
        arb_id = (msg_type << 26) | (device_id << 16) | (topic_id & 0xFFFF)
        return arb_id
    
    def _parse_arbitration_id(self, arb_id: int) -> tuple:
        """Parse CAN arbitration ID"""
        msg_type = (arb_id >> 26) & 0x07
        device_id = (arb_id >> 16) & 0xFF
        topic_id = arb_id & 0xFFFF
        return MessageType(msg_type), device_id, topic_id
    
    def _send_message(self, msg_type: MessageType, topic_id: int, data: bytes):
        """Send a CAN message"""
        if not self.bus or not self.running:
            return
        
        arb_id = self._build_arbitration_id(msg_type, self.device_id, topic_id)
        msg = can.Message(
            arbitration_id=arb_id,
            data=data,
            is_extended_id=True
        )
        
        try:
            self.bus.send(msg)
        except Exception as e:
            logger.error(f"Failed to send message: {e}")
    
    def _send_discovery(self):
        """Send discovery message"""
        data = struct.pack('B', self.device_id)
        self._send_message(MessageType.DISCOVERY, 0, data)
    
    def _heartbeat_loop(self):
        """Periodic heartbeat to maintain presence"""
        while self.running:
            self._send_message(MessageType.HEARTBEAT, 0, b'')
            
            # Clean up stale devices (not seen in 5 seconds)
            current_time = time.time()
            with self.lock:
                stale_devices = [
                    dev_id for dev_id, dev in self.devices.items()
                    if dev_id != self.device_id and (current_time - dev.last_seen) > 5.0
                ]
                for dev_id in stale_devices:
                    logger.warning(f"Device {dev_id} timed out")
                    del self.devices[dev_id]
            
            time.sleep(1.0)
    
    def _receive_loop(self):
        """Main receive loop for CAN messages"""
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None:
                    continue
                
                self._process_message(msg)
                
            except Exception as e:
                logger.error(f"Error receiving message: {e}")
    
    def _process_message(self, msg: can.Message):
        """Process incoming CAN message"""
        try:
            msg_type, device_id, topic_id = self._parse_arbitration_id(msg.arbitration_id)
            
            # Ignore our own messages
            if device_id == self.device_id:
                return
            
            # Update device presence
            with self.lock:
                if device_id not in self.devices:
                    self.devices[device_id] = DeviceInfo(device_id=device_id)
                self.devices[device_id].last_seen = time.time()
            
            # Handle message types
            if msg_type == MessageType.DISCOVERY:
                logger.info(f"Device {device_id} discovered")
                
            elif msg_type == MessageType.HEARTBEAT:
                pass  # Already updated last_seen
                
            elif msg_type == MessageType.CLAIM_PUB:
                self._handle_claim_pub(device_id, topic_id, msg.data)
                
            elif msg_type == MessageType.CLAIM_SUB:
                self._handle_claim_sub(device_id, topic_id, msg.data)
                
            elif msg_type == MessageType.DATA:
                self._handle_data(device_id, topic_id, msg.data)
                
            elif msg_type == MessageType.ERROR:
                logger.error(f"Error message from device {device_id}: {msg.data}")
                
        except Exception as e:
            logger.error(f"Error processing message: {e}")
    
    def _handle_claim_pub(self, device_id: int, topic_id: int, data: bytes):
        """Handle publisher claim"""
        if len(data) < 1:
            return
        
        data_type = DataType(data[0])
        
        with self.lock:
            if topic_id in self.topics:
                topic = self.topics[topic_id]
                if topic.publisher_device is not None and topic.publisher_device != device_id:
                    logger.error(
                        f"Conflict! Device {device_id} trying to publish topic {topic_id} "
                        f"already owned by device {topic.publisher_device}"
                    )
                    # Send error message
                    error_data = struct.pack('HB', topic_id, 1)  # Conflict error
                    self._send_message(MessageType.ERROR, topic_id, error_data)
                    return
            else:
                self.topics[topic_id] = TopicInfo(topic_id=topic_id, data_type=data_type)
            
            self.topics[topic_id].publisher_device = device_id
            self.devices[device_id].published_topics.add(topic_id)
            
        logger.info(f"Device {device_id} publishing topic {topic_id} ({data_type.name})")
    
    def _handle_claim_sub(self, device_id: int, topic_id: int, data: bytes):
        """Handle subscriber claim"""
        if len(data) < 1:
            return
        
        data_type = DataType(data[0])
        
        with self.lock:
            if topic_id not in self.topics:
                self.topics[topic_id] = TopicInfo(topic_id=topic_id, data_type=data_type)
            
            self.topics[topic_id].subscribers.add(device_id)
            self.devices[device_id].subscribed_topics.add(topic_id)
            
        logger.info(f"Device {device_id} subscribing to topic {topic_id} ({data_type.name})")
    
    def _handle_data(self, device_id: int, topic_id: int, data: bytes):
        """Handle data message"""
        with self.lock:
            if topic_id not in self.topics:
                return
            
            topic = self.topics[topic_id]
            
            # Verify publisher
            if topic.publisher_device != device_id:
                logger.warning(f"Device {device_id} sent data for topic {topic_id} but is not publisher")
                return
            
            # Decode based on data type
            try:
                value = self._decode_value(topic.data_type, data)
                topic.last_value = value
                topic.last_timestamp = time.time()
                
                # Call callbacks
                if topic_id in self.callbacks:
                    for callback in self.callbacks[topic_id]:
                        try:
                            callback(value)
                        except Exception as e:
                            logger.error(f"Error in callback: {e}")
                            
            except Exception as e:
                logger.error(f"Error decoding data for topic {topic_id}: {e}")
    
    def _encode_value(self, data_type: DataType, value: Any) -> bytes:
        """Encode value to bytes based on data type"""
        if data_type == DataType.INT8:
            return struct.pack('b', int(value))
        elif data_type == DataType.INT32:
            return struct.pack('i', int(value))
        elif data_type == DataType.FLOAT:
            return struct.pack('f', float(value))
        elif data_type == DataType.BOOL:
            return struct.pack('?', bool(value))
        elif data_type == DataType.ASCII_STRING:
            return value.encode('ascii')[:8]  # Max 8 bytes for CAN
        else:
            raise ValueError(f"Unknown data type: {data_type}")
    
    def _decode_value(self, data_type: DataType, data: bytes) -> Any:
        """Decode bytes to value based on data type"""
        if data_type == DataType.INT8:
            return struct.unpack('b', data[:1])[0]
        elif data_type == DataType.INT32:
            return struct.unpack('i', data[:4])[0]
        elif data_type == DataType.FLOAT:
            return struct.unpack('f', data[:4])[0]
        elif data_type == DataType.BOOL:
            return struct.unpack('?', data[:1])[0]
        elif data_type == DataType.ASCII_STRING:
            return data.decode('ascii').rstrip('\x00')
        else:
            raise ValueError(f"Unknown data type: {data_type}")
    
    def claim_publisher(self, topic_id: int, data_type: DataType):
        """Claim a topic for publishing"""
        with self.lock:
            if topic_id in self.topics:
                topic = self.topics[topic_id]
                if topic.publisher_device is not None and topic.publisher_device != self.device_id:
                    raise RuntimeError(
                        f"Topic {topic_id} already claimed by device {topic.publisher_device}"
                    )
            else:
                self.topics[topic_id] = TopicInfo(topic_id=topic_id, data_type=data_type)
            
            self.topics[topic_id].publisher_device = self.device_id
            self.devices[self.device_id].published_topics.add(topic_id)
        
        # Send claim message
        data = struct.pack('B', data_type)
        self._send_message(MessageType.CLAIM_PUB, topic_id, data)
        
    def claim_subscriber(self, topic_id: int, data_type: DataType):
        """Claim a topic for subscribing"""
        with self.lock:
            if topic_id not in self.topics:
                self.topics[topic_id] = TopicInfo(topic_id=topic_id, data_type=data_type)
            
            self.topics[topic_id].subscribers.add(self.device_id)
            self.devices[self.device_id].subscribed_topics.add(topic_id)
        
        # Send claim message
        data = struct.pack('B', data_type)
        self._send_message(MessageType.CLAIM_SUB, topic_id, data)
    
    def publish_value(self, topic_id: int, value: Any):
        """Publish a value to a topic"""
        with self.lock:
            if topic_id not in self.topics:
                raise RuntimeError(f"Topic {topic_id} not claimed for publishing")
            
            topic = self.topics[topic_id]
            if topic.publisher_device != self.device_id:
                raise RuntimeError(f"Device {self.device_id} is not publisher of topic {topic_id}")
            
            data = self._encode_value(topic.data_type, value)
            topic.last_value = value
            topic.last_timestamp = time.time()
        
        self._send_message(MessageType.DATA, topic_id, data)
    
    def get_value(self, topic_id: int) -> Optional[Any]:
        """Get the last value of a topic"""
        with self.lock:
            if topic_id not in self.topics:
                return None
            return self.topics[topic_id].last_value
    
    def subscribe_callback(self, topic_id: int, callback: Callable[[Any], None]):
        """Register a callback for topic updates"""
        with self.lock:
            if topic_id not in self.callbacks:
                self.callbacks[topic_id] = []
            self.callbacks[topic_id].append(callback)


class Publisher:
    """Publisher interface for a specific data type"""
    
    def __init__(self, manager: CANBusManager, data_type: DataType):
        self.manager = manager
        self.data_type = data_type
        self.claimed_topics: set = set()
    
    def __call__(self, topic_id: int):
        """Claim and return a publisher handle"""
        if topic_id not in self.claimed_topics:
            self.manager.claim_publisher(topic_id, self.data_type)
            self.claimed_topics.add(topic_id)
        return PublisherHandle(self.manager, topic_id)


class PublisherHandle:
    """Handle for publishing to a specific topic"""
    
    def __init__(self, manager: CANBusManager, topic_id: int):
        self.manager = manager
        self.topic_id = topic_id
    
    def set(self, value: Any):
        """Publish a value"""
        self.manager.publish_value(self.topic_id, value)


class Subscriber:
    """Subscriber interface for a specific data type"""
    
    def __init__(self, manager: CANBusManager, data_type: DataType):
        self.manager = manager
        self.data_type = data_type
        self.claimed_topics: set = set()
    
    def __call__(self, topic_id: int):
        """Claim and return a subscriber handle"""
        if topic_id not in self.claimed_topics:
            self.manager.claim_subscriber(topic_id, self.data_type)
            self.claimed_topics.add(topic_id)
        return SubscriberHandle(self.manager, topic_id)


class SubscriberHandle:
    """Handle for subscribing to a specific topic"""
    
    def __init__(self, manager: CANBusManager, topic_id: int):
        self.manager = manager
        self.topic_id = topic_id
    
    def get(self) -> Optional[Any]:
        """Get the last value"""
        return self.manager.get_value(self.topic_id)
    
    def on_change(self, callback: Callable[[Any], None]):
        """Register a callback for value changes"""
        self.manager.subscribe_callback(self.topic_id, callback)


class PublishInterface:
    """Publisher interface with data type methods"""
    
    def __init__(self, manager: CANBusManager):
        self._int8 = Publisher(manager, DataType.INT8)
        self._int32 = Publisher(manager, DataType.INT32)
        self._float = Publisher(manager, DataType.FLOAT)
        self._bool = Publisher(manager, DataType.BOOL)
        self._string = Publisher(manager, DataType.ASCII_STRING)
    
    def int8(self, topic_id: int) -> PublisherHandle:
        return self._int8(topic_id)
    
    def int32(self, topic_id: int) -> PublisherHandle:
        return self._int32(topic_id)
    
    def float(self, topic_id: int) -> PublisherHandle:
        return self._float(topic_id)
    
    def bool(self, topic_id: int) -> PublisherHandle:
        return self._bool(topic_id)
    
    def string(self, topic_id: int) -> PublisherHandle:
        return self._string(topic_id)


class SubscribeInterface:
    """Subscriber interface with data type methods"""
    
    def __init__(self, manager: CANBusManager):
        self._int8 = Subscriber(manager, DataType.INT8)
        self._int32 = Subscriber(manager, DataType.INT32)
        self._float = Subscriber(manager, DataType.FLOAT)
        self._bool = Subscriber(manager, DataType.BOOL)
        self._string = Subscriber(manager, DataType.ASCII_STRING)
    
    def int8(self, topic_id: int) -> SubscriberHandle:
        return self._int8(topic_id)
    
    def int32(self, topic_id: int) -> SubscriberHandle:
        return self._int32(topic_id)
    
    def float(self, topic_id: int) -> SubscriberHandle:
        return self._float(topic_id)
    
    def bool(self, topic_id: int) -> SubscriberHandle:
        return self._bool(topic_id)
    
    def string(self, topic_id: int) -> SubscriberHandle:
        return self._string(topic_id)


class BusInterface:
    """Interface for a specific CAN bus"""
    
    def __init__(self, manager: CANBusManager):
        self.manager = manager
        self.publish = PublishInterface(manager)
        self.subscribe = SubscribeInterface(manager)


class CANTable:
    """Main CANTable interface - singleton manager"""
    
    _instances: Dict[int, CANBusManager] = {}
    _lock = threading.Lock()
    
    @classmethod
    def initialize(cls, device_id: int):
        """Initialize CANTable with device ID"""
        cls.device_id = device_id
    
    @classmethod
    def bus(cls, bus_id: int, channel: str = None, bitrate: int = 500000) -> BusInterface:
        """Get or create a bus interface"""
        with cls._lock:
            if bus_id not in cls._instances:
                if channel is None:
                    channel = f'can{bus_id}'
                
                manager = CANBusManager(bus_id, cls.device_id, channel, bitrate)
                manager.start()
                cls._instances[bus_id] = manager
            
            return BusInterface(cls._instances[bus_id])
    
    @classmethod
    def shutdown(cls):
        """Shutdown all buses"""
        with cls._lock:
            for manager in cls._instances.values():
                manager.stop()
            cls._instances.clear()


# Example usage
if __name__ == "__main__":
    # Initialize with unique device ID
    CANTable.initialize(device_id=9)
    
    # Publisher example
    CANTable.bus(0).publish.bool(123).set(True)
    CANTable.bus(0).publish.int32(456).set(12345)
    CANTable.bus(0).publish.float(789).set(3.14159)
    
    # Subscriber example
    value = CANTable.bus(0).subscribe.bool(123).get()
    print(f"Bool value: {value}")
    
    # Subscribe with callback
    def on_value_change(new_value):
        print(f"Value changed to: {new_value}")
    
    CANTable.bus(0).subscribe.int32(456).on_change(on_value_change)
    
    # Keep running
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        CANTable.shutdown()