"""
IM ESP32-FRC-devkit Controller using CANTable Middleware API

Uses the middleware's raw CAN frame API to communicate with ESP32
following the FRC CAN protocol.
"""

import tkinter as tk
from tkinter import ttk

# Import the middleware
from can_api import CANTable, FRCCANHelper

# =========================
#   FRC CAN Configuration
# =========================
DEVICE_TYPE = 0x0A        # Device type ID
MANUFACTURER = 0x08       # Manufacturer ID
DEFAULT_DEVICE_NUM = 9    # Device number (0-63)

# API Class IDs (from ESP32 protocol)
API_RX_CONTROL = 0x185    # RoboRIO -> ESP32: [R, G, B, relay, 0, 0, 0, 0]
API_TX_INPUTS = 0x195     # ESP32 -> RoboRIO: [ain_lo, ain_hi, btnA, btnB, 0, 0, 0, 0]
API_TX_RESET = 0x196      # ESP32 -> RoboRIO: [reset_device, 0, 0, 0, 0, 0, 0, 0]

# PC's device ID for middleware
PC_DEVICE_ID = 0x01

# Topic IDs (arbitrary, just need to be unique)
TOPIC_CONTROL = 1000
TOPIC_INPUTS = 2000
TOPIC_RESET = 2001


class CANTester(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32-FRC Controller (Middleware API)")
        self.geometry("720x500")
        self.minsize(680, 450)

        # State
        self.device_number = tk.IntVar(value=DEFAULT_DEVICE_NUM)
        self.r_val = tk.IntVar(value=0)
        self.g_val = tk.IntVar(value=0)
        self.b_val = tk.IntVar(value=0)
        self.relay_on = tk.BooleanVar(value=False)

        self._run_timers = True
        
        # Initialize middleware
        CANTable.initialize(device_id=PC_DEVICE_ID)
        self.can = CANTable.bus(0, channel='can0', bitrate=1000000)
        
        # Setup publishers and subscribers
        self._setup_can_topics()

        # Build UI
        self._build_ui()

        # Start periodic sender
        self.after(100, self._send_control_periodic)

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _setup_can_topics(self):
        """Setup CAN topics using middleware API"""
        # Get current device number
        dn = self.device_number.get()
        
        # Create FRC CAN IDs
        self.control_can_id = FRCCANHelper.make_id(DEVICE_TYPE, MANUFACTURER, API_RX_CONTROL, dn)
        self.inputs_can_id = FRCCANHelper.make_id(DEVICE_TYPE, MANUFACTURER, API_TX_INPUTS, dn)
        
        # Publisher for control messages (PC -> ESP32)
        self.pub_control = self.can.publish.raw(TOPIC_CONTROL, self.control_can_id, extended=True)
        
        # Subscribers for input messages (ESP32 -> PC)
        self.sub_inputs = self.can.subscribe.raw(TOPIC_INPUTS, self.inputs_can_id, extended=True)
        
        # Register callbacks
        self.sub_inputs.on_change(self._on_inputs_received)
        
        print(f"CAN Topics configured for device number {dn}")
        print(f"  Control (TX): 0x{self.control_can_id:08X}")
        print(f"  Inputs (RX):  0x{self.inputs_can_id:08X}")

    def _reconfigure_topics(self):
        """Reconfigure topics when device number changes"""
        # Note: In a real implementation, you'd need to unsubscribe from old topics
        # For now, we'll just update the CAN IDs and create new publishers/subscribers
        self._setup_can_topics()

    def _send_control_frame(self):
        """Send control frame using middleware API"""
        r = max(0, min(255, int(self.r_val.get())))
        g = max(0, min(255, int(self.g_val.get())))
        b = max(0, min(255, int(self.b_val.get())))
        relay = 1 if self.relay_on.get() else 0
        
        # Build data payload: [R, G, B, relay, 0, 0, 0, 0]
        data = bytes([r, g, b, relay, 0, 0, 0, 0])
        
        # Send via middleware
        self.pub_control.send(data)

    def _on_inputs_received(self, data: bytes):
        """Callback when ESP32 sends input data"""
        if len(data) >= 4:
            # Parse: [ain_lo, ain_hi, btnA, btnB, ...]
            ain_lo = data[0]
            ain_hi = data[1]
            ain_value = ain_lo | (ain_hi << 8)
            btn_a = data[2]
            btn_b = data[3]
            
            # Update UI on main thread
            self.after(0, self._update_inputs_ui, ain_value, btn_a, btn_b)


    def _update_inputs_ui(self, ain_value: int, btn_a: int, btn_b: int):
        """Update UI with ESP32 input values (thread-safe)"""
        # Update analog input
        self.ain_label.config(text=str(ain_value))
        
        # Update button A (active low: 0=pressed, 1=released)
        if btn_a == 0:
            self.btnA_label.config(text="PRESSED", foreground="red", font=("", 10, "bold"))
        else:
            self.btnA_label.config(text="Released", foreground="gray", font=("", 10))
        
        # Update button B
        if btn_b == 0:
            self.btnB_label.config(text="PRESSED", foreground="red", font=("", 10, "bold"))
        else:
            self.btnB_label.config(text="Released", foreground="gray", font=("", 10))

    def _build_ui(self):
        pad = {"padx": 10, "pady": 8}

        # Device number selector
        top = ttk.Frame(self)
        top.pack(fill="x", **pad)
        
        ttk.Label(top, text="Device Number (0-63):").pack(side="left")
        dn_spin = ttk.Spinbox(top, from_=0, to=63, textvariable=self.device_number, 
                              width=5, command=self._on_device_change)
        dn_spin.pack(side="left", padx=6)
        ttk.Button(top, text="Apply", command=self._on_device_change).pack(side="left", padx=5)
        
        ttk.Label(top, text="|").pack(side="left", padx=10)
        ttk.Label(top, text=f"FRC Protocol: Type=0x{DEVICE_TYPE:02X} Mfr=0x{MANUFACTURER:02X}", 
                 font=("", 9)).pack(side="left")
        
        # Status indicator
        self.status_dot = ttk.Label(top, text="● Active", foreground="green", font=("", 9, "bold"))
        self.status_dot.pack(side="right", padx=10)

        # LED sliders
        sliders = ttk.LabelFrame(self, text="LED Control (API 0x185: RGB + Relay)")
        sliders.pack(fill="x", **pad)

        self._add_slider(sliders, "Red", self.r_val)
        self._add_slider(sliders, "Green", self.g_val)
        self._add_slider(sliders, "Blue", self.b_val)

        # Relay
        relay_frame = ttk.LabelFrame(self, text="Relay Control")
        relay_frame.pack(fill="x", **pad)
        
        relay_inner = ttk.Frame(relay_frame)
        relay_inner.pack(padx=10, pady=10)
        
        ttk.Button(relay_inner, text="Toggle Relay", 
                   command=self._toggle_relay, width=18).pack(side="left")
        self.relay_label = ttk.Label(relay_inner, text="OFF", 
                                     foreground="red", font=("", 12, "bold"))
        self.relay_label.pack(side="left", padx=20)

        # Inputs from ESP32
        rb = ttk.LabelFrame(self, text="ESP32 Inputs (API 0x195: Real-time)")
        rb.pack(fill="x", **pad)

        # Analog
        analog_frame = ttk.Frame(rb)
        analog_frame.pack(fill="x", padx=10, pady=8)
        ttk.Label(analog_frame, text="Analog Input:", width=15).pack(side="left")
        self.ain_label = ttk.Label(analog_frame, text="0", 
                                   font=("", 12, "bold"), foreground="blue")
        self.ain_label.pack(side="left")
        ttk.Label(analog_frame, text="(12-bit: 0-4095)").pack(side="left", padx=10)

        # Buttons
        btn_row = ttk.Frame(rb)
        btn_row.pack(fill="x", padx=10, pady=8)
        
        ttk.Label(btn_row, text="Button A:", width=12).pack(side="left")
        self.btnA_label = ttk.Label(btn_row, text="Released", width=10, foreground="gray")
        self.btnA_label.pack(side="left", padx=5)
        
        ttk.Label(btn_row, text="Button B:", width=12).pack(side="left", padx=(20, 0))
        self.btnB_label = ttk.Label(btn_row, text="Released", width=10, foreground="gray")
        self.btnB_label.pack(side="left", padx=5)

        # Middleware info
        mw_frame = ttk.LabelFrame(self, text="Middleware API Usage")
        mw_frame.pack(fill="x", **pad)
        
        mw_text = (
            "Publisher: CANTable.bus(0).publish.raw(topic_id, can_id).send(data)\n"
            "Subscriber: CANTable.bus(0).subscribe.raw(topic_id, can_id).on_change(callback)\n"
            "FRC CAN ID: FRCCANHelper.make_id(device_type, mfr, api_class, dev_num)"
        )
        ttk.Label(mw_frame, text=mw_text, font=("Courier", 8), 
                 justify="left").pack(padx=8, pady=6)

        # Protocol details
        proto = ttk.LabelFrame(self, text="CAN Protocol")
        proto.pack(fill="x", **pad)
        
        proto_text = (
            f"TX 0x185: [R, G, B, relay, 0, 0, 0, 0]  →  CAN ID: 0x{self.control_can_id:08X}\n"
            f"RX 0x195: [ain_lo, ain_hi, btnA, btnB, 0, 0, 0, 0]  ←  CAN ID: 0x{self.inputs_can_id:08X}"
        )
        self.proto_label = ttk.Label(proto, text=proto_text, font=("Courier", 8), 
                                     justify="left")
        self.proto_label.pack(padx=8, pady=6)

    def _add_slider(self, parent, name, var):
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=8, pady=6)
        ttk.Label(row, text=name, width=8).pack(side="left")
        scale = ttk.Scale(row, from_=0, to=255, variable=var, orient="horizontal")
        scale.pack(side="left", fill="x", expand=True, padx=6)
        val_lbl = ttk.Label(row, textvariable=var, width=4, font=("", 10, "bold"))
        val_lbl.pack(side="right")

    def _toggle_relay(self):
        """Toggle relay state"""
        new_state = not self.relay_on.get()
        self.relay_on.set(new_state)
        
        self.relay_label.configure(
            text="ON" if new_state else "OFF",
            foreground="green" if new_state else "red"
        )

    def _on_device_change(self):
        """Called when device number changes"""
        dn = max(0, min(63, self.device_number.get()))
        self.device_number.set(dn)
        
        # Reconfigure topics with new device number
        self._reconfigure_topics()
        
        # Update protocol display
        proto_text = (
            f"TX 0x185: [R, G, B, relay, 0, 0, 0, 0]  →  CAN ID: 0x{self.control_can_id:08X}\n"
            f"RX 0x195: [ain_lo, ain_hi, btnA, btnB, 0, 0, 0, 0]  ←  CAN ID: 0x{self.inputs_can_id:08X}"
        )
        self.proto_label.config(text=proto_text)

    def _send_control_periodic(self):
        """Send control frame every 100ms (10 Hz)"""
        if not self._run_timers:
            return
        
        self._send_control_frame()
        self.after(100, self._send_control_periodic)

    def _on_close(self):
        """Clean shutdown"""
        self._run_timers = False
        self.after(200, self._finalize)

    def _finalize(self):
        """Final cleanup"""
        try:
            CANTable.shutdown()
        except Exception as e:
            print(f"Shutdown error: {e}")
        self.destroy()


if __name__ == "__main__":
    print("ESP32-FRC Controller via Middleware API")
    print("=" * 50)
    print(f"Device Type: 0x{DEVICE_TYPE:02X}")
    print(f"Manufacturer: 0x{MANUFACTURER:02X}")
    print(f"Default Device Number: {DEFAULT_DEVICE_NUM}")
    print(f"PC Device ID: 0x{PC_DEVICE_ID:02X}")
    print()
    
    app = CANTester()
    app.mainloop()