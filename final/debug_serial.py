#!/usr/bin/env python3
"""Debug serial communication with RP2350"""

import serial
import time

port = '/dev/ttyACM0'
baudrate = 115200

print(f"Opening {port} at {baudrate} baud...")

try:
    ser = serial.Serial(port, baudrate, timeout=1)
    print("✓ Serial port opened")
    
    time.sleep(2)  # Wait for reset
    
    print("\nListening for startup messages...")
    for i in range(10):
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"RX: {line}")
        time.sleep(0.2)
    
    print("\nSending ping command...")
    ser.write(b'{"cmd":"ping"}\n')
    
    print("Waiting for response...")
    for i in range(10):
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"RX: {line}")
        time.sleep(0.2)
    
    ser.close()
    print("\n✓ Test complete")
    
except serial.SerialException as e:
    print(f"✗ Error: {e}")
except KeyboardInterrupt:
    print("\n✗ Interrupted")