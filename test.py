from __future__ import annotations
import socket
from queue import Queue

import struct
from typing import Dict


HOST = "192.168.1.184"
PORTS = range(1, 65536)

# Your CPU can handle this
THREAD_COUNT = 2000

tcp_queue = Queue()
udp_queue = Queue()


def tcp_worker():
    while True:
        port = tcp_queue.get()
        if port is None:
            break
        tcp_probe(port)
        tcp_queue.task_done()


def udp_worker():
    while True:
        port = udp_queue.get()
        if port is None:
            break
        udp_listen(port)
        udp_queue.task_done()


def tcp_probe(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.15)
    try:
        s.connect((HOST, port))
        print(f"[TCP:{port}] CONNECTED — waiting for data...")
        s.settimeout(1)
        while True:
            data = s.recv(4096)
            if not data:
                break
            print(f"[TCP:{port}] {data.hex()} | {data}")
    except:
        pass
    finally:
        s.close()


def udp_listen(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("", port))
        s.settimeout(1)
    except:
        return
    try:
        data, addr = s.recvfrom(4096)
        print(f"[UDP:{port}] From {addr}: {data.hex()} | {data}")
    except:
        pass
    finally:
        s.close()

cRequestRestartCode = 0x04
cRequestReboot = 0x08
cRequestNormal = 0x00
cTagCommVersion = 0x01
cFMSCommVersion = 0x00
cTeleoperated = 0x00
cTest = 0x01
cAutonomous = 0x02
cEnabled = 0x04
cEmergencyStop = 0x80
cFMSConnected = 0x08
cFMSRadioPing = 0x10
cFMSRobotPing = 0x08
cFMSRobotComms = 0x20
cRTagCANInfo = 0x0e
cRTagCPUInfo = 0x05
cRTagRAMInfo = 0x06
cRTagDiskInfo = 0x04
cRequestTime = 0x01
cRobotHasCode = 0x20
cTagDate = 0x0f
cTagJoystick = 0x0c
cTagTimezone = 0x10
cRed1 = 0x00
cRed2 = 0x01
cRed3 = 0x02
cBlue1 = 0x03
cBlue2 = 0x04
cBlue3 = 0x05
class RobotTelemetryParser:
    """
    Best-effort parser for incoming robot-to-DS UDP packets.
    Assumptions:
      - header layout: seq(2) | comm_ver(1) | control(1) | request(1) | alliance(1)
        (This matches the DS->RIO format; robot replies commonly reuse tag framing afterward.)
      - tags: size(1) | id(1) | data(size-1)
    Interpretation rules:
      - If tag data is 4 bytes -> try float32
      - If tag data is 2 bytes -> try uint16 (or int16 for signed values)
      - Else -> raw hex
    """
    def parse(self, data: bytes) -> Dict[str, object]:
        out: Dict[str, object] = {
            "seq": None,
            "comm_version": None,
            "control": None,
            "request": None,
            "alliance": None,
            "tags": []
        }
        if len(data) < 6:
            out["raw"] = data.hex()
            return out
        # header
        seq = struct.unpack_from(">H", data, 0)[0]
        comm_ver = data[2]
        control = data[3]
        request = data[4]
        alliance = data[5]
        out.update({"seq": seq, "comm_version": comm_ver, "control": control, "request": request, "alliance": alliance})
        # parse tags from offset 6
        offset = 6
        tags = []
        while offset + 2 <= len(data):
            size = data[offset]
            if size == 0:
                break
            if offset + 1 >= len(data):
                break
            tag_id = data[offset + 1]
            payload_len = size - 1
            payload_start = offset + 2
            payload_end = payload_start + payload_len
            if payload_end > len(data):
                # truncated; include what we have and stop
                payload = data[payload_start:len(data)]
                offset = len(data)
            else:
                payload = data[payload_start:payload_end]
                offset = payload_end
            interpreted = self._interpret_tag(tag_id, payload)
            tags.append({"id": tag_id, "raw": payload.hex(), "parsed": interpreted})
        out["tags"] = tags
        return out

    def _interpret_tag(self, tag_id: int, payload: bytes):
        # CPU info tag (cRTagCPUInfo): often floats or percent. Try float32
        if tag_id == cRTagCPUInfo and len(payload) >= 4:
            try:
                val = struct.unpack(">f", payload[:4])[0]
                return {"type": "cpu_percent", "value": val}
            except Exception:
                pass
        if tag_id == cRTagRAMInfo and len(payload) >= 4:
            try:
                val = struct.unpack(">f", payload[:4])[0]
                return {"type": "ram_percent", "value": val}
            except Exception:
                pass
        if tag_id == cRTagDiskInfo and len(payload) >= 4:
            try:
                val = struct.unpack(">f", payload[:4])[0]
                return {"type": "disk_percent", "value": val}
            except Exception:
                pass
        if tag_id == cRTagCANInfo and len(payload) >= 2:
            # try uint16 for CAN utilization/errors
            try:
                val = struct.unpack(">H", payload[:2])[0]
                return {"type": "can_metric_uint16", "value": val}
            except Exception:
                pass
        # Generic fallbacks:
        if len(payload) == 4:
            # try float32
            try:
                f = struct.unpack(">f", payload)[0]
                return {"type": "float32", "value": f}
            except Exception:
                pass
        if len(payload) == 2:
            try:
                u = struct.unpack(">H", payload)[0]
                return {"type": "uint16", "value": u}
            except Exception:
                pass
        # otherwise raw
        return {"type": "raw", "value": payload.hex()}


def main():
    # print("Starting scan...\n")

    # # Start workers
    # for _ in range(THREAD_COUNT):
    #     threading.Thread(target=tcp_worker, daemon=True).start()
    #     threading.Thread(target=udp_worker, daemon=True).start()

    # # Fill queues
    # for port in PORTS:
    #     tcp_queue.put(port)
    #     udp_queue.put(port)

    # # Wait
    # tcp_queue.join()
    # udp_queue.join()
    # print("\nFinished scanning all ports.")
    parcer = RobotTelemetryParser()
    print(parcer.parse(bytes.fromhex('05e90104220b9c00')))


if __name__ == "__main__":
    main()
