#!/usr/bin/env python3
"""
Simple Motor Controller Test

Direct test of TMC2209 motor controller with simple commands.
Press Ctrl+C at any time to stop.

Usage:
    python3 motor_controller.py

Commands:
    right  - Move right (both motors forward)
    left   - Move left (both motors reverse)
    up     - Move up (motors opposite)
    down   - Move down (motors opposite reverse)
    stop   - Stop all motors
    quit   - Exit

Author: Air Hockey Team
Date: December 3, 2025
"""

import serial
import json
import time
import sys
import serial.tools.list_ports


def find_motor_controller():
    """Find XIAO ESP32-S3."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == 0x303A and port.pid == 0x1001:
            return port.device
    return None


class SimpleMotorController:
    """Simple motor controller for testing."""
    
    def __init__(self, port=None):
        if port is None:
            port = find_motor_controller()
            if port is None:
                print("ERROR: Could not find XIAO ESP32-S3")
                sys.exit(1)
        
        self.port = port
        self.ser = None
        self.default_speed = 30.0  # Default RPM
    
    def connect(self):
        """Connect to motor controller."""
        try:
            print(f"Connecting to {self.port}...")
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            time.sleep(2)  # Wait for ESP32 to boot
            
            # Clear buffer
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line and not line.startswith('{'):
                    print(f"  {line}")
            
            # Test connection
            response = self.send_cmd({"cmd": "ping"})
            if response and response.get('status') == 'ok':
                print(f"✓ Connected to motor controller")
                return True
            else:
                print("✗ Ping failed")
                return False
                
        except Exception as e:
            print(f"✗ Connection error: {e}")
            return False
    
    def send_cmd(self, cmd_dict):
        """Send JSON command and get response."""
        try:
            cmd_json = json.dumps(cmd_dict) + '\n'
            self.ser.write(cmd_json.encode('utf-8'))
            self.ser.flush()
            
            # Read response (skip debug messages)
            for _ in range(5):  # Try up to 5 lines
                if self.ser.in_waiting or self._wait_data():
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            resp = json.loads(line)
                            # Skip debug messages
                            if 'debug' not in resp:
                                return resp
                        except:
                            pass
            return None
        except Exception as e:
            print(f"Command error: {e}")
            return None
    
    def _wait_data(self, timeout=0.5):
        """Wait for serial data."""
        start = time.time()
        while (time.time() - start) < timeout:
            if self.ser.in_waiting:
                return True
            time.sleep(0.01)
        return False
    
    def enable(self):
        """Enable motors."""
        print("Enabling motors...")
        resp = self.send_cmd({"cmd": "enable", "state": True, "motor": "both"})
        if resp and resp.get('status') == 'ok':
            print("✓ Motors enabled")
            return True
        print("✗ Enable failed")
        return False
    
    def disable(self):
        """Disable motors."""
        resp = self.send_cmd({"cmd": "enable", "state": False, "motor": "both"})
        if resp:
            print("✓ Motors disabled")
    
    def stop(self):
        """Stop all motors."""
        self.send_cmd({"cmd": "stop", "motor": "both"})
    
    def move(self, r_rpm, l_rpm):
        """Move both motors at specified RPM."""
        cmd = {"cmd": "move", "motor": "both", "r_rpm": r_rpm, "l_rpm": l_rpm}
        print(f"  Sending command: {cmd}")
        resp = self.send_cmd(cmd)
        print(f"  Response: {resp}")
        return resp
    
    # CoreXY directional commands
    # Motor 1 (Right) = X + Y
    # Motor 2 (Left)  = X - Y
    
    def right(self, speed=None):
        """Move right (pure X+)."""
        rpm = speed if speed else self.default_speed
        print(f"→ RIGHT at {rpm} RPM")
        print(f"   CoreXY: Motor1(R)={rpm}, Motor2(L)={rpm}")
        # For pure X movement, both motors must spin the same direction
        resp = self.move(rpm, rpm)
        if resp and resp.get('status') == 'ok':
            print("  ✓ Moving")
        else:
            print(f"  ✗ Failed: {resp}")
    
    def left(self, speed=None):
        """Move left (pure X-)."""
        rpm = speed if speed else self.default_speed
        print(f"← LEFT at {rpm} RPM")
        print(f"   CoreXY: Motor1(R)={-rpm}, Motor2(L)={-rpm}")
        resp = self.move(-rpm, -rpm)
        if resp and resp.get('status') == 'ok':
            print("  ✓ Moving")
        else:
            print(f"  ✗ Failed: {resp}")
    
    def up(self, speed=None):
        """Move up (pure Y+)."""
        rpm = speed if speed else self.default_speed
        print(f"↑ UP at {rpm} RPM")
        print(f"   CoreXY: Motor1(R)={-rpm}, Motor2(L)={rpm}")
        # Swapped: Motor 1 reverse, Motor 2 forward
        resp = self.move(-rpm, rpm)
        if resp and resp.get('status') == 'ok':
            print("  ✓ Moving")
        else:
            print(f"  ✗ Failed: {resp}")
    
    def down(self, speed=None):
        """Move down (pure Y-)."""
        rpm = speed if speed else self.default_speed
        print(f"↓ DOWN at {rpm} RPM")
        print(f"   CoreXY: Motor1(R)={rpm}, Motor2(L)={-rpm}")
        # Swapped: Motor 1 forward, Motor 2 reverse
        resp = self.move(rpm, -rpm)
        if resp and resp.get('status') == 'ok':
            print("  ✓ Moving")
        else:
            print(f"  ✗ Failed: {resp}")
    
    def test_individual(self):
        """Test individual motors to diagnose wiring."""
        print("\n=== INDIVIDUAL MOTOR TEST ===")
        print("Watch CAREFULLY which direction the gantry moves\n")
        
        print("TEST 1: Motor 1 ONLY at +30 RPM")
        print("  Command: r_rpm=30, l_rpm=0")
        self.move(30, 0)
        time.sleep(2)
        status = self.send_cmd({"cmd": "status"})
        print(f"  Status: {status}")
        direction1 = input("  Which direction did it move? (e.g., 'right-down', 'left-up', 'none'): ").strip()
        self.stop()
        time.sleep(1)
        
        print("\nTEST 2: Motor 2 ONLY at +30 RPM")
        print("  Command: r_rpm=0, l_rpm=30")
        self.move(0, 30)
        time.sleep(2)
        status = self.send_cmd({"cmd": "status"})
        print(f"  Status: {status}")
        direction2 = input("  Which direction did it move? (e.g., 'right-up', 'left-down', 'none'): ").strip()
        self.stop()
        time.sleep(1)
        
        print("\nTEST 3: BOTH motors at +30 RPM")
        print("  Command: r_rpm=30, l_rpm=30")
        self.move(30, 30)
        time.sleep(2)
        status = self.send_cmd({"cmd": "status"})
        print(f"  Status: {status}")
        direction3 = input("  Which direction did it move? (should be pure X): ").strip()
        self.stop()
        time.sleep(1)
        
        print("\nTEST 4: Motors OPPOSITE (r=30, l=-30)")
        print("  Command: r_rpm=30, l_rpm=-30")
        self.move(30, -30)
        time.sleep(2)
        status = self.send_cmd({"cmd": "status"})
        print(f"  Status: {status}")
        direction4 = input("  Which direction did it move? (should be pure Y): ").strip()
        self.stop()
        
        print("\n" + "="*60)
        print("DIAGNOSIS:")
        print("="*60)
        print(f"Test 1 (Motor1 only): {direction1}")
        print(f"Test 2 (Motor2 only): {direction2}")
        print(f"Test 3 (Both forward): {direction3}")
        print(f"Test 4 (Opposite):     {direction4}")
        print("\nIf Test 1 or Test 2 showed 'none', that motor is NOT working!")
        print("If both tests showed diagonal movement, CoreXY is working correctly.")
        print("="*60)
    
    def close(self):
        """Close connection."""
        if self.ser and self.ser.is_open:
            self.stop()
            self.disable()
            self.ser.close()
            print("Disconnected")



def main():
    """Main program."""
    print("="*60)
    print("SIMPLE MOTOR CONTROLLER TEST")
    print("="*60)
    print("\nCommands:")
    print("  right  - Move right (pure X)")
    print("  left   - Move left (pure X)")
    print("  up     - Move up (pure Y)")
    print("  down   - Move down (pure Y)")
    print("  stop   - Stop motors")
    print("  test   - Test individual motors (diagnose wiring)")
    print("  diag   - Show diagnostic info")
    print("  quit   - Exit")
    print("\nPress Ctrl+C at any time to emergency stop and exit")
    print("="*60 + "\n")
    
    controller = SimpleMotorController()
    
    try:
        # Connect
        if not controller.connect():
            print("Failed to connect. Exiting.")
            return
        
        # Enable motors
        if not controller.enable():
            print("Failed to enable motors. Exiting.")
            return
        
        print("\n✓ Ready! Type commands below:\n")
        
        # Main loop
        while True:
            try:
                cmd = input(">>> ").strip().lower()
                
                if not cmd:
                    continue
                
                if cmd in ['quit', 'exit', 'q']:
                    break
                    
                elif cmd == 'right':
                    controller.right()
                    
                elif cmd == 'left':
                    controller.left()
                    
                elif cmd == 'up':
                    controller.up()
                    
                elif cmd == 'down':
                    controller.down()
                    
                elif cmd == 'stop':
                    print("⏹ STOP")
                    controller.stop()
                
                elif cmd == 'test':
                    controller.test_individual()
                
                elif cmd == 'diag':
                    print("\n=== DIAGNOSTICS ===")
                    diag = controller.send_cmd({"cmd": "diag"})
                    if diag:
                        import json
                        print(json.dumps(diag, indent=2))
                    else:
                        print("Failed to get diagnostics")
                    
                else:
                    print(f"Unknown command: {cmd}")
                    
            except KeyboardInterrupt:
                print("\n\n⚠ Ctrl+C - Emergency stop!")
                controller.stop()
                break
                
            except EOFError:
                print("\nEOF - Exiting")
                break
    
    finally:
        print("\nShutting down...")
        controller.close()
        print("Done.")


if __name__ == "__main__":
    main()
