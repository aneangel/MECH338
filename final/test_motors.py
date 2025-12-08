#!/usr/bin/env python3
"""
Motor diagnostic test - checks all aspects of motor control with live pin monitoring
"""

import serial
import json
import time
import sys
import threading

port = '/dev/ttyACM1'
baudrate = 115200

def send_cmd(ser, cmd_dict):
    """Send command and get response"""
    cmd_json = json.dumps(cmd_dict) + '\n'
    ser.write(cmd_json.encode())
    ser.flush()
    
    # Wait for response
    start = time.time()
    while time.time() - start < 1.0:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                try:
                    return json.loads(line)
                except:
                    pass
        time.sleep(0.01)
    return None

def read_pins_continuously(ser, stop_event, motor_filter=None):
    """Read and display pin states in real-time"""
    step_count = {'m1': 0, 'm2': 0}
    last_step1 = None
    last_step2 = None
    
    while not stop_event.is_set():
        resp = send_cmd(ser, {"cmd": "read_pins"})
        if resp and resp.get('status') == 'ok':
            # Count STEP transitions
            if last_step1 is not None and resp['STEP1_STATE'] != last_step1:
                step_count['m1'] += 1
            if last_step2 is not None and resp['STEP2_STATE'] != last_step2:
                step_count['m2'] += 1
            
            last_step1 = resp['STEP1_STATE']
            last_step2 = resp['STEP2_STATE']
            
            # Clear line and print
            sys.stdout.write('\r' + ' '*100 + '\r')
            
            if motor_filter == 'M1' or motor_filter is None:
                sys.stdout.write(f"M1: STEP={resp['STEP1_STATE']} DIR={resp['DIR1_STATE']} EN={resp['EN1_STATE']} (steps:{step_count['m1']})  ")
            if motor_filter == 'M2' or motor_filter is None:
                sys.stdout.write(f"M2: STEP={resp['STEP2_STATE']} DIR={resp['DIR2_STATE']} EN={resp['EN2_STATE']} (steps:{step_count['m2']})")
            
            sys.stdout.flush()
        time.sleep(0.05)  # 20Hz update rate

print("="*60)
print("MOTOR DIAGNOSTIC TEST")
print("="*60)

try:
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"✓ Connected to {port}")
    time.sleep(2)
    
    # Test 1: Ping
    print("\n[Test 1] Ping controller...")
    resp = send_cmd(ser, {"cmd": "ping"})
    print(f"Response: {resp}")
    
    # Test 2: Get diagnostics
    print("\n[Test 2] Get diagnostics...")
    resp = send_cmd(ser, {"cmd": "diag"})
    if resp:
        print(f"Motors enabled: M1={resp.get('motor1_enabled')}, M2={resp.get('motor2_enabled')}")
        print(f"Step intervals: M1={resp.get('stepInterval1_us')} us, M2={resp.get('stepInterval2_us')} us")
        print(f"Mode: {resp.get('mode')}")
    
    # Test 3: Enable motors
    print("\n[Test 3] Enabling motors...")
    resp = send_cmd(ser, {"cmd": "enable", "state": True})
    print(f"Response: {resp}")
    time.sleep(0.5)
    
    # Check if enabled
    resp = send_cmd(ser, {"cmd": "diag"})
    if resp:
        print(f"Motors enabled: M1={resp.get('motor1_enabled')}, M2={resp.get('motor2_enabled')}")
    
    # Test 4: Motor 1 with live pin monitoring
    print("\n[Test 4] Testing Motor 1 at 10 RPM with live pin monitoring...")
    print("Watch the STEP pin toggle count - it should increase rapidly")
    print("Starting in 2 seconds...\n")
    time.sleep(2)
    
    # Start pin monitoring thread
    stop_monitor = threading.Event()
    monitor_thread = threading.Thread(target=read_pins_continuously, args=(ser, stop_monitor, 'M1'))
    monitor_thread.daemon = True
    monitor_thread.start()
    
    time.sleep(0.5)
    resp = send_cmd(ser, {"cmd": "move", "motor": "R", "rpm": 10.0})
    
    print("\nMotor 1 running... Press ENTER to stop")
    input()
    
    send_cmd(ser, {"cmd": "stop"})
    time.sleep(0.5)
    stop_monitor.set()
    monitor_thread.join(timeout=1)
    print("\n")
    
    # Test 5: Motor 2 with live pin monitoring
    print("\n[Test 5] Testing Motor 2 at 10 RPM with live pin monitoring...")
    print("Watch the STEP pin toggle count - it should increase rapidly")
    print("Starting in 2 seconds...\n")
    time.sleep(2)
    
    # Start pin monitoring thread
    stop_monitor = threading.Event()
    monitor_thread = threading.Thread(target=read_pins_continuously, args=(ser, stop_monitor, 'M2'))
    monitor_thread.daemon = True
    monitor_thread.start()
    
    time.sleep(0.5)
    resp = send_cmd(ser, {"cmd": "move", "motor": "L", "rpm": 10.0})
    
    print("\nMotor 2 running... Press ENTER to stop")
    input()
    
    send_cmd(ser, {"cmd": "stop"})
    time.sleep(0.5)
    stop_monitor.set()
    monitor_thread.join(timeout=1)
    print("\n")
    
    # Test 6: Both motors with velocity command
    print("\n[Test 6] Testing velocity command (X direction, both motors)...")
    print("Monitoring both motors...")
    print("Starting in 2 seconds...\n")
    time.sleep(2)
    
    # Start pin monitoring thread
    stop_monitor = threading.Event()
    monitor_thread = threading.Thread(target=read_pins_continuously, args=(ser, stop_monitor, None))
    monitor_thread.daemon = True
    monitor_thread.start()
    
    time.sleep(0.5)
    resp = send_cmd(ser, {"cmd": "velocity", "x_vel": 20.0, "y_vel": 0.0})
    time.sleep(0.5)
    resp = send_cmd(ser, {"cmd": "velocity", "x_vel": 20.0, "y_vel": 0.0})
    if resp:
        print(f"\nMotor 1 RPM: {resp.get('motor1_rpm')}, Motor 2 RPM: {resp.get('motor2_rpm')}")
    
    print("\nBoth motors running... Press ENTER to stop")
    input()
    
    send_cmd(ser, {"cmd": "stop"})
    time.sleep(0.5)
    stop_monitor.set()
    monitor_thread.join(timeout=1)
    print("\n")
    
    # Test 7: Final diagnostics
    print("\n[Test 7] Final diagnostics...")
    resp = send_cmd(ser, {"cmd": "diag"})
    if resp:
        print(json.dumps(resp, indent=2))
    
    # Disable motors
    print("\nDisabling motors...")
    send_cmd(ser, {"cmd": "enable", "state": False})
    
    ser.close()
    print("\n✓ Test complete")
    
    print("\n" + "="*60)
    print("RESULTS ANALYSIS:")
    print("="*60)
    print("Check the step counts from each test:")
    print("  - If STEP count increased: Signal is working!")
    print("  - If STEP stayed at 0: No pulses generated (firmware issue)")
    print("  - If pulses generated but motor didn't move:")
    print("      • Check EN pin wiring (should be active LOW)")
    print("      • Check motor power (VM = 12-24V to TMC2209)")
    print("      • Check VIO power (3.3V to TMC2209)")
    print("      • Check current setting on TMC2209 (trim pot)")
    print("      • Check motor wiring (coils A1/A2, B1/B2)")
    print("="*60)
    
except KeyboardInterrupt:
    print("\n✗ Interrupted")
    try:
        send_cmd(ser, {"cmd": "stop"})
        send_cmd(ser, {"cmd": "enable", "state": False})
    except:
        pass
except Exception as e:
    print(f"✗ Error: {e}")
