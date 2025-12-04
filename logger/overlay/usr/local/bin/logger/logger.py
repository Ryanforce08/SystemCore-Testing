#!/usr/bin/env python3
import os
import json
import time
import socket
import struct
import threading
import http.server
import socketserver
from datetime import datetime
from html import escape


DEVICE_TYPE_MAP = {
    0: "Broadcast", 1: "Robot Controller", 2: "Motor Controller", 3: "Relay Controller",
    4: "Gyro Sensor", 5: "Accelerometer", 6: "Ultrasonic Sensor", 7: "Gear Tooth Sensor",
    8: "PDP", 9: "PCM", 10: "Misc", 11: "IO Breakout", 12: "Servo",
    31: "Firmware Update"
}

MANUFACTURER_MAP = {
    0: "Broadcast", 1: "NI", 2: "Luminary", 3: "DEKA", 4: "CTRE",
    5: "REV", 6: "Grapple", 7: "MindSensors", 8: "Team Use",
    9: "Kauai Labs", 10: "Copperforge"
}

# roboRIO heartbeat CAN ID
HEARTBEAT_ID = 0x01011840

can_messages = {}              # decoded + grouped
can_raw_log = []              # raw log for "show all"
decoded_heartbeat = None
last_heartbeat_time = 0

raw_mode = False              # user toggle

MAX_RAW_LOG = 500             # limit memory usage

current_bus = 0
bus_switch_event = threading.Event()
bus_lock = threading.Lock()



def parse_frc_id(can_id):
    """Decompose the 29-bit CAN ID into components."""
    device_type = (can_id >> 24) & 0x1F
    manufacturer = (can_id >> 16) & 0xFF
    api_id = (can_id >> 6) & 0x3FF
    dev_number = can_id & 0x3F
    return device_type, manufacturer, dev_number, api_id


def decode_heartbeat(payload):
    """Decode the roboRIO FRC heartbeat message."""
    if len(payload) < 8:
        return None

    bits = ''.join(f'{b:08b}' for b in payload)

    def b(s, l):
        return int(bits[s:s+l], 2)

    return {
        "match_time": b(56, 8),
        "match_number": b(46, 10),
        "replay_number": b(40, 6),
        "red_alliance": b(39, 1),
        "enabled": b(38, 1),
        "autonomous": b(37, 1),
        "test": b(36, 1),
        "watchdog": b(35, 1),
        "tournament_type": b(32, 3),
        "year": 2000 + b(26, 6) - 36,
        "month": b(22, 4) + 1,
        "day": b(17, 5),
        "seconds": b(11, 6),
        "minutes": b(5, 6),
        "hours": b(0, 5),
    }


def format_ts(d):
    return f"{d['year']:04}-{d['month']:02}-{d['day']:02} {d['hours']:02}:{d['minutes']:02}:{min(d['seconds'], 59):02}"

def can_reader():
    global last_heartbeat_time, decoded_heartbeat
    global can_messages, raw_mode, current_bus

    sock = None
    bus_name = None

    while True:
        # Check if we need to switch buses
        if bus_switch_event.is_set() or sock is None:
            if sock:
                sock.close()

            with bus_lock:
                bus_num = current_bus
                bus_name = f"can_s{bus_num}"
            
            try:
                print(f"[CAN] Switching to {bus_name}")
                sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                
                # Enable loopback to see TX from this device
                sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 1)
                
                # Set timeout so we can check for bus switches periodically
                sock.settimeout(0.5)
                
                sock.bind((bus_name,))
                print(f"[CAN] Successfully bound to {bus_name}")
                bus_switch_event.clear()
            except Exception as e:
                print(f"[CAN] Error binding to {bus_name}: {e}")
                time.sleep(1)
                continue

        try:
            frame = sock.recv(16)
        except socket.timeout:
            # Timeout allows us to check for bus switches
            continue
        except Exception as e:
            print(f"[CAN] Receive error: {e}")
            sock = None
            continue

        can_id, dlc, data = parse_can_frame(frame)

        # record raw
        can_raw_log.append({
            "id": f"0x{can_id:08X}",
            "dlc": dlc,
            "data": ' '.join(f"{x:02X}" for x in data),
            "time": time.time()
        })
        if len(can_raw_log) > MAX_RAW_LOG:
            can_raw_log.pop(0)

        # heartbeat
        if can_id == HEARTBEAT_ID:
            try:
                decoded_heartbeat = decode_heartbeat(data)
                last_heartbeat_time = time.time()
            except:
                pass

        # decode FRC ID
        dev_type, manufacturer, dev_number, api_id = parse_frc_id(can_id)

        can_messages[can_id] = {
            "id": f"0x{can_id:08X}",
            "device_type": DEVICE_TYPE_MAP.get(dev_type, f"Unknown {dev_type}"),
            "manufacturer": MANUFACTURER_MAP.get(manufacturer, f"Unknown {manufacturer}"),
            "dev_number": dev_number,
            "api_id": f"0x{api_id:03X}",
            "data_hex": ' '.join(f"{x:02X}" for x in data),
            "ts": time.time()
        }



def parse_can_frame(frame):
    can_id = int.from_bytes(frame[0:4], "little")
    dlc = frame[4]
    data = list(frame[8:8+dlc])
    return can_id, dlc, data


threading.Thread(target=can_reader, daemon=True).start()


class Handler(http.server.BaseHTTPRequestHandler):

    def do_GET(self):
        global raw_mode

        if self.path == "/":
            self.respond_html(index_html())
            return

        if self.path == "/api/messages":
            if raw_mode:
                self.respond_json(can_raw_log)
            else:
                self.respond_json(list(can_messages.values()))
            return
        


        if self.path == "/api/heartbeat":
            hb = {
                "present": decoded_heartbeat is not None,
                "age": time.time() - last_heartbeat_time if last_heartbeat_time else None,
                "decoded": decoded_heartbeat,
            }
            self.respond_json(hb)
            return

        if self.path.startswith("/api/set_raw?"):
            raw_mode = ("true" in self.path)
            self.respond_json({"ok": True, "raw": raw_mode})
            return
        
        if self.path.startswith("/api/set_bus?"):
            try:
                num = int(self.path.split("=", 1)[1])
                if 0 <= num <= 4:
                    with bus_lock:
                        global current_bus
                        current_bus = num
                    # Clear the message tables when switching buses
                    can_messages.clear()
                    can_raw_log.clear()
                    bus_switch_event.set()
                    self.respond_json({"ok": True, "bus": f"can{num}"})
                else:
                    self.send_error(400, "Bus must be 0–4")
            except Exception as e:
                self.send_error(400, f"Invalid bus: {e}")
            return

        self.send_error(404)

    def respond_json(self, obj):
        js = json.dumps(obj, indent=2)
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(js.encode())

    def respond_html(self, html):
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(html.encode())

    def log_message(self, a, *b):
        return  # silence logs


def index_html():
    return """
<!DOCTYPE html>
<html>
<head>
    <title>SocketCAN Viewer</title>
    <style>
        * { box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #333333;
            color: #e0e0e0;
            margin: 0;
            padding: 20px;
        }
        h2 {
            color: #ffffff;
            margin-top: 0;
            border-bottom: 2px solid #2196F3;
            padding-bottom: 10px;
        }
        table { 
            border-collapse: collapse;
            width: 100%;
            margin-top: 10px;
            background: #2a2a2a;
            border-radius: 8px;
            overflow: hidden;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        th, td { 
            border: 1px solid #444;
            padding: 10px 12px;
            font-size: 13px;
            text-align: left;
        }
        th { 
            background: #1a1a1a;
            color: #2196F3;
            font-weight: 600;
            text-transform: uppercase;
            font-size: 12px;
            letter-spacing: 0.5px;
        }
        tr:hover {
            background: #3a3a3a;
        }
        #heartbeat { 
            padding: 15px;
            margin-bottom: 15px;
            background: #2a2a2a;
            border-radius: 8px;
            border-left: 4px solid #2196F3;
            font-weight: 500;
            box-shadow: 0 2px 4px rgba(0,0,0,0.3);
        }
        #controls { 
            margin-bottom: 15px;
        }
        button { 
            padding: 10px 18px;
            margin-right: 8px;
            margin-bottom: 8px;
            background: #444;
            color: #e0e0e0;
            border: 1px solid #555;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
            transition: all 0.2s;
        }
        button:hover {
            background: #555;
            border-color: #666;
        }
        button:active {
            transform: translateY(1px);
        }
        #busBtns { 
            margin: 15px 0;
            padding: 15px;
            background: #2a2a2a;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.3);
        }
        #busBtns strong {
            color: #ffffff;
            display: block;
            margin-bottom: 10px;
        }
        .active-bus { 
            background: #2196F3;
            color: white;
            border-color: #2196F3;
            font-weight: 600;
        }
        .active-bus:hover {
            background: #42A5F5;
            border-color: #42A5F5;
        }
    </style>
</head>
<body>

<h2>CAN Viewer (SocketCAN) - TX & RX</h2>

<div id="controls">
    <button onclick="setRaw(false)">Decoded FRC Mode</button>
    <button onclick="setRaw(true)">Raw Mode (all CAN traffic)</button>
</div>

<div id="heartbeat">Loading heartbeat...</div>

<div id="busBtns">
    <strong>Select CAN Bus:</strong><br>
    <button id="btn0" onclick="setBus(0)">CAN 0</button>
    <button id="btn1" onclick="setBus(1)">CAN 1</button>
    <button id="btn2" onclick="setBus(2)">CAN 2</button>
    <button id="btn3" onclick="setBus(3)">CAN 3</button>
    <button id="btn4" onclick="setBus(4)">CAN 4</button>
</div>

<table id="tbl">
    <thead>
        <tr>
            <th>ID</th><th>Device</th><th>Manufacturer</th><th>Number</th><th>API</th><th>Data</th><th>Age</th>
        </tr>
    </thead>
    <tbody></tbody>
</table>

<script>
let rawMode = false;
let currentBus = 0;

function setBus(n) {
    fetch("/api/set_bus?num=" + n)
        .then(r => r.json())
        .then(data => {
            if (data.ok) {
                currentBus = n;
                updateBusButtons();
                console.log("Switched to " + data.bus);
            }
        });
}

function updateBusButtons() {
    for (let i = 0; i <= 4; i++) {
        const btn = document.getElementById("btn" + i);
        if (i === currentBus) {
            btn.classList.add("active-bus");
        } else {
            btn.classList.remove("active-bus");
        }
    }
}

function setRaw(v) {
    rawMode = v;
    fetch("/api/set_raw?" + (v ? "true" : "false"));
}

function update() {
    fetch("/api/messages")
        .then(r => r.json())
        .then(rows => {
            const tb = document.querySelector("#tbl tbody");
            tb.innerHTML = "";
            rows.forEach(r => {
                const tr = document.createElement("tr");
                tr.innerHTML = `
                    <td>${r.id}</td>
                    <td>${r.device_type || ""}</td>
                    <td>${r.manufacturer || ""}</td>
                    <td>${r.dev_number || ""}</td>
                    <td>${r.api_id || ""}</td>
                    <td>${r.data_hex}</td>
                    <td>${(Date.now()/1000 - r.ts).toFixed(2)}s</td>
                `;
                tb.appendChild(tr);
            });
        });

    fetch("/api/heartbeat")
        .then(r => r.json())
        .then(h => {
            const d = document.getElementById("heartbeat");
            if (!h.present) {
                d.innerHTML = "No Heartbeat";
                return;
            }
            let s = h.decoded;
            d.innerHTML = `
                Heartbeat: ${s.enabled ? "ENABLED" : "DISABLED"}
                | ${s.autonomous ? "AUTO" : "TELEOP"}
                | Match ${s.match_number}
                | Time Left ${s.match_time}s
            `;
        });
}

updateBusButtons();
setInterval(update, 200);
</script>

</body>
</html>
"""


# =====================================================
#                      START SERVER
# =====================================================

def main():
    PORT = 9001
    httpd = socketserver.TCPServer(("", PORT), Handler)
    print("Serving on port", PORT)
    httpd.serve_forever()


if __name__ == "__main__":
    main()