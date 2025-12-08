#!/usr/bin/env python3
"""
Read and display pin states from RP2350 motor controller
"""

import serial
import json
import time
import sys

PORT = '/dev/ttyACM1'
BAUD = 115200

def find_port():
    """Try common ports"""
    ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
    for port in ports:
        try:
            ser = serial.Serial(port, BAUD, timeout=1)
            time.sleep(0.5)
            return ser, port
        except:
            continue
    return None, None

def send_command(ser, cmd):
    """Send JSON command and get response"""
    ser.write((json.dumps(cmd) + '\n').encode())
    time.sleep(0.1)
    
    response = ser.readline().decode().strip()
    try:
        return json.loads(response)
    except:
        return None

def main():
    print("=== RP2350 Pin State Reader ===\n")
    
    ser, port = find_port()
    if not ser:
        print("ERROR: Could not find RP2350 on any port")
        return
    
    print(f"Connected to {port}")
    print("Waiting for board to initialize...")
    time.sleep(1.5)
    
    # Clear any startup messages
    while ser.in_waiting:
        ser.readline()
    
    # Read initial state
    print("\nReading pin states...")
    resp = send_command(ser, {"cmd": "read_pins"})
    
    if resp and resp.get('status') == 'ok':
        print("\n--- Motor 1 (Right) ---")
        print(f"  STEP Pin {resp['STEP1_PIN']}: {resp['STEP1_STATE']} ({'HIGH' if resp['STEP1_STATE'] else 'LOW'})")
        print(f"  DIR Pin  {resp['DIR1_PIN']}: {resp['DIR1_STATE']} ({'HIGH' if resp['DIR1_STATE'] else 'LOW'})")
        print(f"  EN Pin   {resp['EN1_PIN']}: {resp['EN1_STATE']} ({'HIGH' if resp['EN1_STATE'] else 'LOW'}) - {'ENABLED' if resp['EN1_ACTIVE'] else 'DISABLED'}")
        
        print("\n--- Motor 2 (Left) ---")
        print(f"  STEP Pin {resp['STEP2_PIN']}: {resp['STEP2_STATE']} ({'HIGH' if resp['STEP2_STATE'] else 'LOW'})")
        print(f"  DIR Pin  {resp['DIR2_PIN']}: {resp['DIR2_STATE']} ({'HIGH' if resp['DIR2_STATE'] else 'LOW'})")
        print(f"  EN Pin   {resp['EN2_PIN']}: {resp['EN2_STATE']} ({'HIGH' if resp['EN2_STATE'] else 'LOW'}) - {'ENABLED' if resp['EN2_ACTIVE'] else 'DISABLED'}")
        
        print("\n--- Notes ---")
        print("  - EN pins are active LOW (LOW = enabled, HIGH = disabled)")
        print("  - STEP pins should toggle when motor is moving")
        print("  - DIR pins set rotation direction")
        
        # Try enabling motors and reading again
        print("\n\nEnabling motors...")
        resp = send_command(ser, {"cmd": "enable", "state": True})
        if resp:
            print(f"Response: {resp.get('message', 'OK')}")
        
        time.sleep(0.2)
        
        print("\nReading pin states after enable...")
        resp = send_command(ser, {"cmd": "read_pins"})
        
        if resp and resp.get('status') == 'ok':
            print("\n--- Motor 1 (Right) ---")
            print(f"  EN Pin {resp['EN1_PIN']}: {resp['EN1_STATE']} ({'HIGH' if resp['EN1_STATE'] else 'LOW'}) - {'ENABLED' if resp['EN1_ACTIVE'] else 'DISABLED'}")
            
            print("\n--- Motor 2 (Left) ---")
            print(f"  EN Pin {resp['EN2_PIN']}: {resp['EN2_STATE']} ({'HIGH' if resp['EN2_STATE'] else 'LOW'}) - {'ENABLED' if resp['EN2_ACTIVE'] else 'DISABLED'}")
            
            if resp['EN1_ACTIVE'] and resp['EN2_ACTIVE']:
                print("\n✓ Both motors are now enabled!")
            else:
                print("\n✗ Motors are NOT enabled - check hardware connections")
                print("  - Verify EN pins are connected to driver EN inputs")
                print("  - Ensure VIO is connected (3.3V logic supply to driver)")
    else:
        print("ERROR: Failed to read pins")
        if resp:
            print(f"Response: {resp}")
    
    ser.close()

if __name__ == "__main__":
    main()
