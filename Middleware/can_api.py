"""
CAN Bus Middleware with FRC Protocol Support
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
    RAW_BYTES = 5  # For custom protocols like FRC


class MessageType(IntEnum):
    """CAN message types for protocol"""
    DISCOVERY = 0x00
    CLAIM_PUB = 0x01
    CLAIM_SUB = 0x02
    DATA = 0x03
    HEARTBEAT = 0x04
    ERROR = 0x05
    RAW = 0x06  # Raw CAN frame (bypass protocol)


@dataclass
class TopicInfo:
    """Information about a topic"""
    topic_id: int
    data_type: DataType
    publisher_device: Optional[int] = None
    last_value: Any = None
    last_timestamp: float = 0.0
    subscribers: set = field(default_factory=set)
    custom_can_id: Optional[int] = None  # For FRC protocol
    use_extended_id: bool = True


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
        
        # Protocol mode
        self.protocol_mode = "auto"  # "auto", "middleware", "raw"
        
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
    
    def _heartbeat_loop(self):
        """Periodic heartbeat to maintain presence"""
        while self.running:
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
            # Check if this is a raw/custom protocol message
            with self.lock:
                # Look for topics with custom CAN IDs
                for topic_id, topic in self.topics.items():
                    if topic.custom_can_id is not None and topic.custom_can_id == msg.arbitration_id:
                        # This is a message for a subscribed raw topic
                        if self.device_id in topic.subscribers:
                            self._handle_raw_data(topic_id, msg.data)
                        return
            
        except Exception as e:
            logger.error(f"Error processing message: {e}")
    
    def _handle_raw_data(self, topic_id: int, data: bytes):
        """Handle raw data message"""
        with self.lock:
            if topic_id not in self.topics:
                return
            
            topic = self.topics[topic_id]
            topic.last_value = data
            topic.last_timestamp = time.time()
            
            # Call callbacks
            if topic_id in self.callbacks:
                for callback in self.callbacks[topic_id]:
                    try:
                        callback(data)
                    except Exception as e:
                        logger.error(f"Error in callback: {e}")
    
    def claim_publisher_raw(self, topic_id: int, custom_can_id: int, use_extended_id: bool = True):
        """Claim a topic for publishing raw CAN frames"""
        with self.lock:
            if topic_id in self.topics:
                topic = self.topics[topic_id]
                if topic.publisher_device is not None and topic.publisher_device != self.device_id:
                    raise RuntimeError(
                        f"Topic {topic_id} already claimed by device {topic.publisher_device}"
                    )
            else:
                self.topics[topic_id] = TopicInfo(
                    topic_id=topic_id, 
                    data_type=DataType.RAW_BYTES,
                    custom_can_id=custom_can_id,
                    use_extended_id=use_extended_id
                )
            
            self.topics[topic_id].publisher_device = self.device_id
            self.devices[self.device_id].published_topics.add(topic_id)
    
    def claim_subscriber_raw(self, topic_id: int, custom_can_id: int, use_extended_id: bool = True):
        """Claim a topic for subscribing to raw CAN frames"""
        with self.lock:
            if topic_id not in self.topics:
                self.topics[topic_id] = TopicInfo(
                    topic_id=topic_id, 
                    data_type=DataType.RAW_BYTES,
                    custom_can_id=custom_can_id,
                    use_extended_id=use_extended_id
                )
            
            self.topics[topic_id].subscribers.add(self.device_id)
            self.devices[self.device_id].subscribed_topics.add(topic_id)
    
    def publish_raw(self, topic_id: int, data: bytes):
        """Publish raw CAN frame"""
        with self.lock:
            if topic_id not in self.topics:
                raise RuntimeError(f"Topic {topic_id} not claimed for publishing")
            
            topic = self.topics[topic_id]
            if topic.publisher_device != self.device_id:
                raise RuntimeError(f"Device {self.device_id} is not publisher of topic {topic_id}")
            
            if topic.custom_can_id is None:
                raise RuntimeError(f"Topic {topic_id} has no custom CAN ID")
            
            topic.last_value = data
            topic.last_timestamp = time.time()
            
            # Send raw CAN frame
            msg = can.Message(
                arbitration_id=topic.custom_can_id,
                is_extended_id=topic.use_extended_id,
                data=data
            )
        
        try:
            self.bus.send(msg)
        except Exception as e:
            logger.error(f"Failed to send raw message: {e}")
    
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


class RawPublisher:
    """Publisher for raw CAN frames"""
    
    def __init__(self, manager: CANBusManager):
        self.manager = manager
        self.claimed_topics: Dict[int, int] = {}  # topic_id -> can_id
    
    def __call__(self, topic_id: int, can_id: int, extended: bool = True):
        """Claim and return a raw publisher handle"""
        if topic_id not in self.claimed_topics:
            self.manager.claim_publisher_raw(topic_id, can_id, extended)
            self.claimed_topics[topic_id] = can_id
        return RawPublisherHandle(self.manager, topic_id)


class RawPublisherHandle:
    """Handle for publishing raw CAN frames"""
    
    def __init__(self, manager: CANBusManager, topic_id: int):
        self.manager = manager
        self.topic_id = topic_id
    
    def send(self, data: bytes):
        """Send raw CAN frame"""
        self.manager.publish_raw(self.topic_id, data)


class RawSubscriber:
    """Subscriber for raw CAN frames"""
    
    def __init__(self, manager: CANBusManager):
        self.manager = manager
        self.claimed_topics: Dict[int, int] = {}
    
    def __call__(self, topic_id: int, can_id: int, extended: bool = True):
        """Claim and return a raw subscriber handle"""
        if topic_id not in self.claimed_topics:
            self.manager.claim_subscriber_raw(topic_id, can_id, extended)
            self.claimed_topics[topic_id] = can_id
        return RawSubscriberHandle(self.manager, topic_id)


class RawSubscriberHandle:
    """Handle for subscribing to raw CAN frames"""
    
    def __init__(self, manager: CANBusManager, topic_id: int):
        self.manager = manager
        self.topic_id = topic_id
    
    def get(self) -> Optional[bytes]:
        """Get the last received data"""
        return self.manager.get_value(self.topic_id)
    
    def on_change(self, callback: Callable[[bytes], None]):
        """Register a callback for data changes"""
        self.manager.subscribe_callback(self.topic_id, callback)


class PublishInterface:
    """Publisher interface with raw CAN support"""
    
    def __init__(self, manager: CANBusManager):
        self.manager = manager
        self._raw = RawPublisher(manager)
    
    def raw(self, topic_id: int, can_id: int, extended: bool = True) -> RawPublisherHandle:
        """Publish raw CAN frames with custom CAN ID"""
        return self._raw(topic_id, can_id, extended)


class SubscribeInterface:
    """Subscriber interface with raw CAN support"""
    
    def __init__(self, manager: CANBusManager):
        self.manager = manager
        self._raw = RawSubscriber(manager)
    
    def raw(self, topic_id: int, can_id: int, extended: bool = True) -> RawSubscriberHandle:
        """Subscribe to raw CAN frames with custom CAN ID"""
        return self._raw(topic_id, can_id, extended)


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


# Helper for FRC CAN protocol
class FRCCANHelper:
    """Helper for FRC CAN ID construction"""
    
    @staticmethod
    def make_id(device_type: int, manufacturer: int, api_class: int, device_number: int) -> int:
        """
        Create FRC extended CAN ID
        Format: [device_type:8][manufacturer:8][api_class:10][device_number:6]
        """
        return ((device_type & 0xFF) << 24) | \
               ((manufacturer & 0xFF) << 16) | \
               ((api_class & 0x3FF) << 6) | \
               (device_number & 0x3F)
    
    @staticmethod
    def parse_id(can_id: int) -> tuple:
        """Parse FRC CAN ID -> (device_type, manufacturer, api_class, device_number)"""
        device_type = (can_id >> 24) & 0xFF
        manufacturer = (can_id >> 16) & 0xFF
        api_class = (can_id >> 6) & 0x3FF
        device_number = can_id & 0x3F
        return device_type, manufacturer, api_class, device_number


if __name__ == "__main__":
    # Example: FRC protocol usage
    CANTable.initialize(device_id=0x01)
    
    # Create FRC CAN ID
    frc_id = FRCCANHelper.make_id(
        device_type=0x0A,
        manufacturer=0x08,
        api_class=0x185,
        device_number=9
    )
    
    # Publish raw CAN frame
    pub = CANTable.bus(0).publish.raw(topic_id=100, can_id=frc_id)
    pub.send(bytes([255, 128, 0, 1, 0, 0, 0, 0]))  # R, G, B, relay
    
    print("FRC CAN frame sent")
    
    time.sleep(1)
    CANTable.shutdown()