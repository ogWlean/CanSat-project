import serial
import struct
import tkinter as tk
from tkinter import ttk
import threading
import csv
import time

# --- CONFIGURATION ---
SERIAL_PORT = 'COM8'  # Update this to your receiver's actual COM port
BAUD_RATE = 115200  # Must match the transmitter's baud rate

class CanSatDashboard:
    def __init__(self, root):
        self.root = root
        self.root.title("CanSat Ground Station")
        self.root.geometry("600x400")

        # --- UI Variables ---
        self.packet_id = tk.StringVar(value="Waiting for telemetry...")
        self.accel = tk.StringVar(value="Accel Total (m/s²): 0.0")
        self.env = tk.StringVar(value="Environment: 0.0°C | 0.0%")
        self.baro = tk.StringVar(value="Barometer: 0.0 hPa | Alt: 0.0 m")
        
        # --- Build UI Layout ---
        ttk.Label(root, text="🚀 CanSat Telemetry Dashboard", font=("Arial", 16, "bold")).pack(pady=15)
        ttk.Label(root, textvariable=self.packet_id, font=("Arial", 12)).pack(pady=5)
        
        # Data Frame
        frame = ttk.LabelFrame(root, text="Live Sensor Data")
        frame.pack(padx=20, pady=10, fill="both", expand=True)
        
        ttk.Label(frame, textvariable=self.accel, font=("Consolas", 12)).pack(pady=10)
        ttk.Label(frame, textvariable=self.env, font=("Consolas", 12)).pack(pady=10)
        ttk.Label(frame, textvariable=self.baro, font=("Consolas", 12)).pack(pady=10)
        
        # --- CSV Logging Setup ---
        # Creates a unique file for every run based on the current time
        self.csv_filename = f"cansat_flight_data_{int(time.time())}.csv"
        
        # Write the header row
        with open(self.csv_filename, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "Packet ID", "Accel Total", 
                "Temp (C)", "Humidity (%)", 
                "Pressure (hPa)", "Altitude (m)", "CRC-8"
            ])
            
        print(f"Logging telemetry to: {self.csv_filename}")

        # --- Start Serial Thread ---
        # Runs in the background so the UI doesn't freeze
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

    def read_serial(self):
        # Struct format: < (little endian), H (uint16), 5f (5 floats), B (uint8)
        packet_format = '<H5fB' 
        struct_size = struct.calcsize(packet_format) # Must be exactly 23 bytes
        
        try:
            with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
                while True:
                    # Look for framing sync bytes (0xAA, 0x55)
                    if ser.read(1) == b'\xAA':
                        if ser.read(1) == b'\x55':
                            # Sync found, read the rest of the struct
                            data = ser.read(struct_size)
                            if len(data) == struct_size:
                                unpacked = struct.unpack(packet_format, data)
                                # Safely update UI and CSV on the main thread
                                self.root.after(0, self.update_ui, unpacked)
        except Exception as e:
            self.packet_id.set(f"Serial Error: {e}")
            print(f"Serial Error: {e}")

    def update_ui(self, data):
        # --- Unpack Variables ---
        pid = data[0]
        a_tot = data[1]
        temp, hum = data[2:4]
        press, alt = data[4:6]
        crc = data[6] 
        
        # --- Update Screen ---
        self.packet_id.set(f"Packet ID: {pid}  |  CRC-8: {hex(crc)}")
        self.accel.set(f"Accel Total (m/s²): {a_tot:.2f}")
        self.env.set(f"Environment: {temp:.2f}°C | Hum: {hum:.2f}%")
        self.baro.set(f"Barometer: {press:.2f} hPa | Alt: {alt:.2f} m")

        # --- Save to CSV ---
        with open(self.csv_filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                pid, f"{a_tot:.2f}", 
                f"{temp:.2f}", f"{hum:.2f}", f"{press:.2f}", f"{alt:.2f}", hex(crc)
            ])

if __name__ == "__main__":
    root = tk.Tk()
    app = CanSatDashboard(root)
    root.mainloop()