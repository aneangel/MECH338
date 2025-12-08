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
import threading
import serial.tools.list_ports


def find_motor_controller():
    """Find XIAO RP2350 (now used as motor controller in STEP/DIR mode)."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # XIAO RP2350 USB VID/PID
        if port.vid == 0x2886 and port.pid == 0x58:
            return port.device
    # Fallback: Try ESP32-S3 (legacy UART mode - commented out)
    # for port in ports:
    #     if port.vid == 0x303A and port.pid == 0x1001:
    #         return port.device
    return None


def find_ir_sensor_board():
    """Find IR sensor board (separate RP2350 if using two boards)."""
    ports = serial.tools.list_ports.comports()
    # If using a second RP2350 for IR sensors only, detect it here
    # For now, IR sensors are on the same board as motor controller
    # so this function returns None
    return None
    # Uncomment below if you have a dedicated IR sensor board:
    # for port in ports:
    #     if port.vid == 0x2886 and port.pid == 0x58:
    #         # Would need additional logic to distinguish between two RP2350s
    #         return port.device
    # return None


class SimpleMotorController:
    """Simple motor controller for testing."""
    
    def __init__(self, port=None, ir_sensor_port=None, enable_ir_monitor=True):
        if port is None:
            port = find_motor_controller()
            if port is None:
                print("ERROR: Could not find XIAO RP2350 motor controller")
                sys.exit(1)
        
        self.port = port
        self.ser = None
        self.default_speed = 30.0  # Default RPM
        
        # IR sensor board
        self.ir_sensor_port = ir_sensor_port
        self.ir_sensor_ser = None
        self.ir_monitor_thread = None
        self.ir_monitor_running = False
        self.enable_ir_monitor = enable_ir_monitor
        
        # Idle oscillation
        self.idle_oscillation_thread = None
        self.idle_oscillation_running = False
        self.saved_position = (0, 0)  # Store position before oscillation
    
    def connect(self):
        """Connect to motor controller."""
        try:
            print(f"Connecting to motor controller at {self.port}...")
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            time.sleep(2)  # Wait for RP2350 to boot
            
            # Clear buffer
            while self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line and not line.startswith('{'):
                    print(f"  {line}")
            
            # Test connection
            response = self.send_cmd({"cmd": "ping"})
            if response and response.get('status') == 'ok':
                print(f"✓ Connected to motor controller")
            else:
                print("✗ Ping failed")
                return False
            
            # IR sensor monitoring disabled - sensors on same RP2350 board
            # (IR monitoring would need to be handled via JSON commands or separate board)
            if self.enable_ir_monitor and False:  # Disabled for single-board setup
                if self.ir_sensor_port is None:
                    self.ir_sensor_port = find_ir_sensor_board()
                
                if self.ir_sensor_port:
                    print(f"Connecting to IR sensor board at {self.ir_sensor_port}...")
                    try:
                        self.ir_sensor_ser = serial.Serial(self.ir_sensor_port, 115200, timeout=0.1)
                        time.sleep(1)
                        print(f"✓ Connected to IR sensor board")
                        
                        # Start monitoring thread
                        self.ir_monitor_running = True
                        self.ir_monitor_thread = threading.Thread(target=self._monitor_ir_sensors, daemon=True)
                        self.ir_monitor_thread.start()
                        print(f"✓ IR sensor monitoring started")
                    except Exception as e:
                        print(f"⚠ Warning: Could not connect to IR sensor board: {e}")
                else:
                    print(f"⚠ Warning: IR sensor board not found")
            
            return True
                
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
    
    def _monitor_ir_sensors(self):
        """Monitor IR sensor board for STOP and RESET commands."""
        print("[IR Monitor] Thread started")
        while self.ir_monitor_running:
            try:
                if self.ir_sensor_ser and self.ir_sensor_ser.in_waiting:
                    line = self.ir_sensor_ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
                    # Check for goal detection commands
                    if line == "STOP":
                        print("\n🎯 [GOAL DETECTED] Stopping gantry...")
                        self.stop_idle_oscillation()
                        self.stop()
                    elif line == "RESET":
                        print("🔄 [GOAL DETECTED] Resetting to saved position...")
                        # Send position reset to saved position
                        self.send_cmd({"cmd": "home", "x": self.saved_position[0], "y": self.saved_position[1]})
                    elif "GOAL" in line:
                        print(f"\n🎯 {line}")
                    elif "Coin detected! Triggering solenoid" in line:
                        print(f"\n🪙 {line}")
                        print("🎮 [GAME START] Starting circle motion...")
                        self.start_idle_oscillation()
                    elif "IR beam broken" in line:
                        print(f"\n🪙 {line}")
                    
                time.sleep(0.01)  # Small delay to prevent CPU spinning
                
            except Exception as e:
                if self.ir_monitor_running:  # Only print if we're supposed to be running
                    print(f"[IR Monitor] Error: {e}")
                break
        
        print("[IR Monitor] Thread stopped")
    
    def start_idle_oscillation(self, amplitude=20.0, speed=30.0):
        """Start idle circular motion.
        
        Args:
            amplitude: Radius of circle in mm
            speed: Movement speed in mm/s
        """
        if self.idle_oscillation_running:
            return  # Already running
        
        # Save current position
        status = self.send_cmd({"cmd": "status"})
        if status:
            self.saved_position = (status.get('x', 0), status.get('y', 0))
        
        self.idle_oscillation_running = True
        self.idle_oscillation_thread = threading.Thread(
            target=self._circle_loop, 
            args=(amplitude, speed),
            daemon=True
        )
        self.idle_oscillation_thread.start()
        print(f"[Circle Motion] Started (radius={amplitude}mm at {speed}mm/s)")
    
    def stop_idle_oscillation(self):
        """Stop idle circular motion."""
        if not self.idle_oscillation_running:
            return
        
        self.idle_oscillation_running = False
        if self.idle_oscillation_thread and self.idle_oscillation_thread.is_alive():
            self.idle_oscillation_thread.join(timeout=1)
        print("[Circle Motion] Stopped")
    
    def _circle_loop(self, radius, speed):
        """Internal circle motion loop that runs in separate thread.
        
        Moves in a continuous circle pattern.
        """
        import math
        
        # Circle parameters
        num_points = 20  # Number of points in circle
        angle_step = (2 * math.pi) / num_points
        
        angle = 0
        
        while self.idle_oscillation_running:
            try:
                # Calculate circle point
                x_offset = radius * math.cos(angle)
                y_offset = radius * math.sin(angle)
                
                # Convert to velocity (move toward next point)
                next_angle = angle + angle_step
                next_x = radius * math.cos(next_angle)
                next_y = radius * math.sin(next_angle)
                
                # Velocity is direction to next point
                dx = next_x - x_offset
                dy = next_y - y_offset
                
                # Normalize and scale by speed
                magnitude = math.sqrt(dx*dx + dy*dy)
                if magnitude > 0:
                    x_vel = (dx / magnitude) * speed
                    y_vel = (dy / magnitude) * speed
                else:
                    x_vel = 0
                    y_vel = 0
                
                # Send velocity command
                self.send_cmd({"cmd": "velocity", "x_vel": x_vel, "y_vel": y_vel})
                
                # Time to reach next point
                if magnitude > 0:
                    move_time = magnitude / speed
                    time.sleep(move_time)
                
                # Move to next angle
                angle = next_angle
                if angle >= 2 * math.pi:
                    angle = 0  # Complete circle, start over
                
            except Exception as e:
                print(f"[Circle Motion] Error: {e}")
                break
        
        # Stop motors when exiting
        self.stop()
        print("[Circle Motion] Thread stopped")
    
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
        """Move both motors at specified RPM (legacy method)."""
        cmd = {"cmd": "move", "motor": "both", "r_rpm": r_rpm, "l_rpm": l_rpm}
        print(f"  Sending command: {cmd}")
        resp = self.send_cmd(cmd)
        print(f"  Response: {resp}")
        return resp
    
    def velocity(self, x_vel, y_vel):
        """Send velocity command in mm/s (ESP32 handles CoreXY kinematics)."""
        cmd = {"cmd": "velocity", "x_vel": x_vel, "y_vel": y_vel}
        print(f"  Sending command: {cmd}")
        resp = self.send_cmd(cmd)
        print(f"  Response: {resp}")
        return resp
    
    # CoreXY directional commands
    # Motor 1 (Right) = X + Y
    # Motor 2 (Left)  = X - Y
    
    def right(self, speed=None):
        """Move right (pure X+)."""
        vel = speed if speed else 100.0  # mm/s
        print(f"→ RIGHT at {vel} mm/s")
        print(f"   Velocity: x_vel={vel}, y_vel=0 (ESP32 handles CoreXY)")
        resp = self.velocity(vel, 0)
        if resp and resp.get('status') == 'ok':
            print(f"  ✓ Moving (M1={resp.get('motor1_rpm'):.1f} RPM, M2={resp.get('motor2_rpm'):.1f} RPM)")
        else:
            print(f"  ✗ Failed: {resp}")
    
    def left(self, speed=None):
        """Move left (pure X-)."""
        vel = speed if speed else 100.0  # mm/s
        print(f"← LEFT at {vel} mm/s")
        print(f"   Velocity: x_vel={-vel}, y_vel=0 (ESP32 handles CoreXY)")
        resp = self.velocity(-vel, 0)
        if resp and resp.get('status') == 'ok':
            print(f"  ✓ Moving (M1={resp.get('motor1_rpm'):.1f} RPM, M2={resp.get('motor2_rpm'):.1f} RPM)")
        else:
            print(f"  ✗ Failed: {resp}")
    
    def up(self, speed=None):
        """Move up (pure Y+)."""
        vel = speed if speed else 100.0  # mm/s
        print(f"↑ UP at {vel} mm/s")
        print(f"   Velocity: x_vel=0, y_vel={-vel} (ESP32 handles CoreXY)")
        resp = self.velocity(0, -vel)
        if resp and resp.get('status') == 'ok':
            print(f"  ✓ Moving (M1={resp.get('motor1_rpm'):.1f} RPM, M2={resp.get('motor2_rpm'):.1f} RPM)")
        else:
            print(f"  ✗ Failed: {resp}")
    
    def down(self, speed=None):
        """Move down (pure Y-)."""
        vel = speed if speed else 100.0  # mm/s
        print(f"↓ DOWN at {vel} mm/s")
        print(f"   Velocity: x_vel=0, y_vel={vel} (ESP32 handles CoreXY)")
        resp = self.velocity(0, vel)
        if resp and resp.get('status') == 'ok':
            print(f"  ✓ Moving (M1={resp.get('motor1_rpm'):.1f} RPM, M2={resp.get('motor2_rpm'):.1f} RPM)")
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
    
    def calibrate_limits(self):
        """Interactive calibration of workspace limits."""
        print("\n" + "="*60)
        print("WORKSPACE LIMIT CALIBRATION")
        print("="*60)
        print("This will help the controller learn your workspace boundaries.")
        print("\nSTEPS:")
        print("1. Motors will be DISABLED - you can move gantry manually")
        print("2. Move gantry to BOTTOM-LEFT corner (minimum X, minimum Y)")
        print("3. Move gantry to TOP-RIGHT corner (maximum X, maximum Y)")
        print("4. Controller will enforce these limits to prevent crashes")
        print("="*60)
        
        input("\nPress ENTER to begin calibration...")
        
        # Disable motors so user can move manually
        print("\n→ Disabling motors...")
        self.disable()
        
        print("\n" + "="*60)
        print("STEP 1: Set MINIMUM limits (bottom-left corner)")
        print("="*60)
        print("Manually push the gantry to the BOTTOM-LEFT corner.")
        print("This is where X and Y are at their MINIMUM values.")
        input("Press ENTER when gantry is at bottom-left corner...")
        
        # Set current position as zero and mark as minimum
        resp = self.send_cmd({"cmd": "home", "x": 0, "y": 0})
        print(f"  Response: {resp}")
        
        resp = self.send_cmd({"cmd": "calibrate", "action": "set_min"})
        print(f"  ✓ Minimum limits set: {resp}")
        
        print("\n" + "="*60)
        print("STEP 2: Set MAXIMUM limits (top-right corner)")
        print("="*60)
        print("Now manually push the gantry to the TOP-RIGHT corner.")
        print("This is where X and Y are at their MAXIMUM values.")
        input("Press ENTER when gantry is at top-right corner...")
        
        # Measure the distance and set as maximum
        print("\nEnter your workspace dimensions:")
        x_size = input("  X axis (width) in mm [default: 300]: ").strip()
        y_size = input("  Y axis (length) in mm [default: 650]: ").strip()
        
        x_size = float(x_size) if x_size else 300.0
        y_size = float(y_size) if y_size else 650.0
        
        resp = self.send_cmd({"cmd": "home", "x": x_size, "y": y_size})
        print(f"  Response: {resp}")
        
        resp = self.send_cmd({"cmd": "calibrate", "action": "set_max"})
        print(f"  ✓ Maximum limits set: {resp}")
        
        # Set home position at center
        print("\n" + "="*60)
        print("STEP 3: Set HOME position")
        print("="*60)
        print("Push the gantry to where you want the HOME position.")
        print("This is where the gantry will go when starting up.")
        print("Recommended: center of workspace")
        input("Press ENTER when gantry is at desired home position...")
        
        resp = self.send_cmd({"cmd": "home", "x": x_size/2, "y": y_size/2})
        print(f"  Response: {resp}")
        
        resp = self.send_cmd({"cmd": "calibrate", "action": "set_home"})
        print(f"  ✓ Home position set: {resp}")
        
        # Check calibration
        resp = self.send_cmd({"cmd": "calibrate", "action": "status"})
        print("\n" + "="*60)
        print("CALIBRATION COMPLETE!")
        print("="*60)
        print(f"X Range: {resp.get('x_min', 0):.1f} to {resp.get('x_max', 350):.1f} mm")
        print(f"Y Range: {resp.get('y_min', 0):.1f} to {resp.get('y_max', 350):.1f} mm")
        print(f"Home: ({resp.get('home_x', 0):.1f}, {resp.get('home_y', 0):.1f})")
        print(f"Calibrated: {resp.get('calibrated', False)}")
        print("="*60)
        
        # Re-enable motors
        print("\n→ Re-enabling motors...")
        self.enable()
        print("✓ Calibration complete! Motors will now stop at limits.")
    
    def check_calibration(self):
        """Check if workspace is calibrated."""
        resp = self.send_cmd({"cmd": "calibrate", "action": "status"})
        return resp if resp else {}
    
    def go_home(self):
        """Move to home position (user manually pushes gantry)."""
        print("\n→ Going to home position...")
        print("  Motors will be DISABLED")
        print("  Manually push the gantry to the home position")
        
        self.disable()
        input("  Press ENTER when gantry is at home position...")
        
        resp = self.send_cmd({"cmd": "calibrate", "action": "go_home"})
        if resp and resp.get('status') == 'ok':
            print(f"  ✓ Position set to home ({resp.get('x', 0):.1f}, {resp.get('y', 0):.1f})")
        else:
            print(f"  ✗ Failed: {resp}")
        
        self.enable()
    
    def close(self):
        """Close connection."""
        # Stop idle oscillation
        self.stop_idle_oscillation()
        
        # Stop IR monitoring
        if self.ir_monitor_thread:
            self.ir_monitor_running = False
            if self.ir_monitor_thread.is_alive():
                self.ir_monitor_thread.join(timeout=1)
        
        if self.ir_sensor_ser and self.ir_sensor_ser.is_open:
            self.ir_sensor_ser.close()
            print("IR sensor board disconnected")
        
        if self.ser and self.ser.is_open:
            self.stop()
            self.disable()
            self.ser.close()
            print("Motor controller disconnected")



def main():
    """Main program."""
    print("="*60)
    print("SIMPLE MOTOR CONTROLLER TEST")
    print("="*60)
    print("\nCommands:")
    print("  right    - Move right (pure X)")
    print("  left     - Move left (pure X)")
    print("  up       - Move up (pure Y)")
    print("  down     - Move down (pure Y)")
    print("  stop     - Stop motors")
    print("  home     - Go to home position")
    print("  calibrate - Calibrate workspace limits (prevents crashes!)")
    print("  test     - Test individual motors (diagnose wiring)")
    print("  diag     - Show diagnostic info")
    print("  quit     - Exit")
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
        
        # Check calibration status
        cal_status = controller.check_calibration()
        if not cal_status.get('calibrated', False):
            print("\n" + "="*60)
            print("⚠ WARNING: Workspace limits NOT calibrated!")
            print("="*60)
            print("Software endstops are DISABLED until you calibrate.")
            print("This means the gantry could crash into the rails!")
            print("\nOptions:")
            print("  calibrate - Set up workspace limits (RECOMMENDED)")
            print("  continue  - Proceed without endstops (DANGEROUS!)")
            print("="*60)
            choice = input("\nYour choice [calibrate/continue]: ").strip().lower()
            if choice == 'calibrate':
                controller.calibrate_limits()
            elif choice != 'continue':
                print("Exiting for safety.")
                return
        else:
            # Calibrated - offer to go home
            print("\n✓ Workspace calibrated")
            print(f"  X: {cal_status.get('x_min', 0):.1f} to {cal_status.get('x_max', 350):.1f} mm")
            print(f"  Y: {cal_status.get('y_min', 0):.1f} to {cal_status.get('y_max', 350):.1f} mm")
            if cal_status.get('home_set', False):
                print(f"  Home: ({cal_status.get('home_x', 0):.1f}, {cal_status.get('home_y', 0):.1f})")
                choice = input("\nGo to home position? [y/N]: ").strip().lower()
                if choice == 'y':
                    controller.go_home()
        
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
                
                elif cmd == 'home':
                    controller.go_home()
                
                elif cmd == 'calibrate':
                    controller.calibrate_limits()
                
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
