#!/usr/bin/env python3
"""
CoreXY Gantry Movement Test Controller

Tests basic movements: right, left, up, down, and diagonal.
Uses velocity commands to control the CoreXY gantry.
"""

import serial
import json
import time
import sys
from typing import Optional, Dict, Any

class CoreXYController:
    """Controller for CoreXY gantry using velocity commands"""
    
    def __init__(self, port: str = '/dev/ttyACM1', baudrate: int = 115200):
        """Initialize serial connection to Arduino"""
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        
    def connect(self) -> bool:
        """Establish serial connection"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino reset
            
            # Clear buffer
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # Set connected before ping so send_command works
            self.connected = True
            
            # Test connection with ping
            if self.ping():
                print(f"✓ Connected to {self.port}")
                return True
            else:
                print(f"✗ Failed to ping controller")
                self.connected = False
                return False
                
        except serial.SerialException as e:
            print(f"✗ Serial connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.stop()
            self.disable_motors()
            self.ser.close()
            self.connected = False
            print("✓ Disconnected")
    
    def send_command(self, cmd_dict: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Send JSON command and receive response"""
        if not self.connected or not self.ser:
            print("✗ Not connected")
            return None
        
        try:
            cmd_json = json.dumps(cmd_dict) + '\n'
            self.ser.write(cmd_json.encode())
            self.ser.flush()
            
            # Wait for response with timeout
            start = time.time()
            while time.time() - start < 1.0:
                if self.ser.in_waiting:
                    response_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if response_line:
                        try:
                            return json.loads(response_line)
                        except json.JSONDecodeError:
                            pass  # Try next line
                time.sleep(0.01)
            
            return None
            
        except serial.SerialException as e:
            print(f"✗ Command error: {e}")
            return None
    
    def ping(self) -> bool:
        """Test connection"""
        resp = self.send_command({"cmd": "ping"})
        return resp is not None and resp.get("status") == "ok"
    
    def enable_motors(self) -> bool:
        """Enable both motors"""
        resp = self.send_command({"cmd": "enable", "state": True})
        success = resp is not None and resp.get("status") == "ok"
        if success:
            print("✓ Motors enabled")
        return success
    
    def disable_motors(self) -> bool:
        """Disable both motors"""
        resp = self.send_command({"cmd": "enable", "state": False})
        success = resp is not None and resp.get("status") == "ok"
        if success:
            print("✓ Motors disabled")
        return success
    
    def stop(self) -> bool:
        """Stop all movement"""
        resp = self.send_command({"cmd": "stop"})
        success = resp is not None and resp.get("status") == "ok"
        if success:
            print("✓ Stopped")
        return success
    
    def set_velocity(self, x_vel: float, y_vel: float) -> Optional[Dict[str, Any]]:
        """Set X and Y velocities in mm/s"""
        cmd = {
            "cmd": "velocity",
            "x_vel": x_vel,
            "y_vel": y_vel
        }
        resp = self.send_command(cmd)
        
        if resp and resp.get("status") == "ok":
            print(f"✓ Velocity set: X={x_vel:.1f} mm/s, Y={y_vel:.1f} mm/s")
            print(f"  Motor1 (Right): {resp.get('motor1_rpm', 0):.1f} RPM")
            print(f"  Motor2 (Left):  {resp.get('motor2_rpm', 0):.1f} RPM")
            print(f"  Position: ({resp.get('x_pos', 0):.1f}, {resp.get('y_pos', 0):.1f}) mm")
        elif resp and resp.get("status") == "endstop":
            print(f"⚠ Endstop triggered: {resp.get('message')}")
        
        return resp
    
    def get_status(self) -> Optional[Dict[str, Any]]:
        """Get current motor status"""
        return self.send_command({"cmd": "status"})
    
    def get_diag(self) -> Optional[Dict[str, Any]]:
        """Get diagnostic information"""
        return self.send_command({"cmd": "diag"})
    
    # Movement primitives
    def move_right(self, speed: float = 50.0):
        """Move right (+X direction)"""
        print(f"\n→ Moving RIGHT at {speed} mm/s")
        return self.set_velocity(speed, 0)
    
    def move_left(self, speed: float = 50.0):
        """Move left (-X direction)"""
        print(f"\n← Moving LEFT at {speed} mm/s")
        return self.set_velocity(-speed, 0)
    
    def move_up(self, speed: float = 50.0):
        """Move up (+Y direction)"""
        print(f"\n↑ Moving UP at {speed} mm/s")
        return self.set_velocity(0, speed)
    
    def move_down(self, speed: float = 50.0):
        """Move down (-Y direction)"""
        print(f"\n↓ Moving DOWN at {speed} mm/s")
        return self.set_velocity(0, -speed)
    
    def move_diagonal_ur(self, speed: float = 50.0):
        """Move diagonally up-right"""
        print(f"\n↗ Moving UP-RIGHT at {speed} mm/s")
        return self.set_velocity(speed, speed)
    
    def move_diagonal_ul(self, speed: float = 50.0):
        """Move diagonally up-left"""
        print(f"\n↖ Moving UP-LEFT at {speed} mm/s")
        return self.set_velocity(-speed, speed)
    
    def move_diagonal_dr(self, speed: float = 50.0):
        """Move diagonally down-right"""
        print(f"\n↘ Moving DOWN-RIGHT at {speed} mm/s")
        return self.set_velocity(speed, -speed)
    
    def move_diagonal_dl(self, speed: float = 50.0):
        """Move diagonally down-left"""
        print(f"\n↙ Moving DOWN-LEFT at {speed} mm/s")
        return self.set_velocity(-speed, -speed)


def run_movement_test(controller: CoreXYController, duration: float = 2.0, speed: float = 50.0):
    """Run comprehensive movement test"""
    
    print("\n" + "="*60)
    print("COREXY GANTRY MOVEMENT TEST")
    print("="*60)
    
    if not controller.enable_motors():
        print("✗ Failed to enable motors")
        return
    
    time.sleep(0.5)
    
    movements = [
        ("Right", lambda: controller.move_right(speed)),
        ("Left", lambda: controller.move_left(speed)),
        ("Up", lambda: controller.move_up(speed)),
        ("Down", lambda: controller.move_down(speed)),
        ("Diagonal UR", lambda: controller.move_diagonal_ur(speed)),
        ("Diagonal UL", lambda: controller.move_diagonal_ul(speed)),
        ("Diagonal DR", lambda: controller.move_diagonal_dr(speed)),
        ("Diagonal DL", lambda: controller.move_diagonal_dl(speed)),
    ]
    
    try:
        for name, move_func in movements:
            print(f"\n{'─'*60}")
            move_func()
            
            # Run for duration
            time.sleep(duration)
            
            # Stop
            controller.stop()
            time.sleep(0.5)
        
        print("\n" + "="*60)
        print("✓ Movement test complete!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n⚠ Test interrupted by user")
    
    finally:
        controller.stop()
        time.sleep(0.5)
        controller.disable_motors()


def interactive_mode(controller: CoreXYController):
    """Interactive control mode"""
    
    print("\n" + "="*60)
    print("INTERACTIVE CONTROL MODE")
    print("="*60)
    print("\nCommands:")
    print("  w/a/s/d  - Up/Left/Down/Right")
    print("  q/e/z/c  - Diagonal (UL/UR/DL/DR)")
    print("  space    - Stop")
    print("  +/-      - Increase/Decrease speed")
    print("  i        - Get status")
    print("  x        - Exit")
    print("="*60)
    
    speed = 50.0
    
    if not controller.enable_motors():
        print("✗ Failed to enable motors")
        return
    
    print(f"\n✓ Motors enabled (speed: {speed} mm/s)")
    print("Ready for commands...")
    
    try:
        import tty
        import termios
        
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                
                if key == 'x':
                    break
                elif key == 'w':
                    controller.move_up(speed)
                elif key == 's':
                    controller.move_down(speed)
                elif key == 'a':
                    controller.move_left(speed)
                elif key == 'd':
                    controller.move_right(speed)
                elif key == 'q':
                    controller.move_diagonal_ul(speed)
                elif key == 'e':
                    controller.move_diagonal_ur(speed)
                elif key == 'z':
                    controller.move_diagonal_dl(speed)
                elif key == 'c':
                    controller.move_diagonal_dr(speed)
                elif key == ' ':
                    controller.stop()
                elif key == '+' or key == '=':
                    speed = min(200.0, speed + 10.0)
                    print(f"Speed: {speed} mm/s")
                elif key == '-':
                    speed = max(10.0, speed - 10.0)
                    print(f"Speed: {speed} mm/s")
                elif key == 'i':
                    status = controller.get_diag()
                    if status:
                        print(f"\nStatus: {json.dumps(status, indent=2)}")
            
            time.sleep(0.01)
    
    except ImportError:
        print("⚠ Interactive mode requires 'select' module (Linux/Mac only)")
        print("Use test mode instead")
    
    finally:
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        except:
            pass
        
        controller.stop()
        controller.disable_motors()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='CoreXY Gantry Movement Test')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--speed', type=float, default=50.0, help='Movement speed (mm/s)')
    parser.add_argument('--duration', type=float, default=2.0, help='Movement duration per direction (s)')
    parser.add_argument('--mode', choices=['test', 'interactive'], default='test',
                       help='Test mode: automated test or interactive control')
    
    args = parser.parse_args()
    
    controller = CoreXYController(port=args.port)
    
    if not controller.connect():
        print("✗ Failed to connect to controller")
        sys.exit(1)
    
    try:
        if args.mode == 'test':
            run_movement_test(controller, duration=args.duration, speed=args.speed)
        else:
            interactive_mode(controller)
    
    finally:
        controller.disconnect()