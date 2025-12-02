"""
Jetson Orin Nano Motor Controller Interface

Communicates with XIAO ESP32-S3 TMC2209 motor controller via USB serial.
Sends JSON commands to control dual stepper motors with RPM-based control.

Usage:
    python motor_controller.py
"""

import serial
import json
import time
from typing import Optional, Dict, Any

class MotorController:
    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize motor controller communication.
        
        Args:
            port: Serial port (e.g., '/dev/ttyACM0', '/dev/ttyUSB0')
            baudrate: Communication speed (default: 115200)
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        
    def connect(self) -> bool:
        """Establish serial connection to controller."""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            time.sleep(2)  # Wait for Arduino to reset
            
            # Clear any startup messages
            while self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                print(f"Startup: {line}")
            
            # Test connection
            response = self.ping()
            if response and response.get('status') == 'ok':
                print(f"Connected to motor controller on {self.port}")
                return True
            else:
                print("Connection failed: No valid ping response")
                return False
                
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.stop_all()  # Safety: stop motors before disconnecting
            self.serial.close()
            print("Disconnected from motor controller")
    
    def send_command(self, command: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Send JSON command to controller and wait for response.
        
        Args:
            command: Dictionary containing command data
            
        Returns:
            Response dictionary or None if failed
        """
        if not self.serial or not self.serial.is_open:
            print("Serial port not open")
            return None
        
        try:
            # Send command as JSON
            json_str = json.dumps(command)
            self.serial.write((json_str + '\n').encode('utf-8'))
            self.serial.flush()
            
            # Wait for response
            if self.serial.in_waiting or self._wait_for_data(timeout=2.0):
                response_line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if response_line:
                    return json.loads(response_line)
            
            return None
            
        except Exception as e:
            print(f"Command failed: {e}")
            return None
    
    def _wait_for_data(self, timeout: float = 1.0) -> bool:
        """Wait for data to be available on serial port."""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.serial.in_waiting:
                return True
            time.sleep(0.01)
        return False
    
    # ===== Motor Control Commands =====
    
    def enable_motors(self, state: bool = True, motor: str = "both") -> Optional[Dict]:
        """Enable or disable motors."""
        cmd = {
            "cmd": "enable",
            "state": state,
            "motor": motor
        }
        return self.send_command(cmd)
    
    def set_rpm(self, motor: str, rpm: float) -> Optional[Dict]:
        """
        Set RPM for a single motor.
        
        Args:
            motor: "R" (right) or "L" (left)
            rpm: Revolutions per minute (positive or negative for direction)
        """
        cmd = {
            "cmd": "move",
            "motor": motor,
            "rpm": rpm
        }
        return self.send_command(cmd)
    
    def set_both_rpm(self, right_rpm: float, left_rpm: float) -> Optional[Dict]:
        """
        Set RPM for both motors simultaneously.
        
        Args:
            right_rpm: RPM for right motor
            left_rpm: RPM for left motor
        """
        cmd = {
            "cmd": "move",
            "motor": "both",
            "r_rpm": right_rpm,
            "l_rpm": left_rpm
        }
        return self.send_command(cmd)
    
    def stop(self, motor: str = "both") -> Optional[Dict]:
        """Stop motors."""
        cmd = {
            "cmd": "stop",
            "motor": motor
        }
        return self.send_command(cmd)
    
    def stop_all(self) -> Optional[Dict]:
        """Emergency stop all motors."""
        return self.stop("both")
    
    def set_motor_current(self, current: int, motor: str = "both") -> Optional[Dict]:
        """
        Set motor current in mA.
        
        Args:
            current: Current in milliamps (e.g., 800)
            motor: "R", "L", or "both"
        """
        cmd = {
            "cmd": "setcurrent",
            "current": current,
            "motor": motor
        }
        return self.send_command(cmd)
    
    def get_status(self) -> Optional[Dict]:
        """Get current status of both motors."""
        cmd = {"cmd": "status"}
        return self.send_command(cmd)
    
    def ping(self) -> Optional[Dict]:
        """Test connection to controller."""
        cmd = {"cmd": "ping"}
        return self.send_command(cmd)


def main():
    """Example usage of motor controller."""
    
    # Create controller instance
    controller = MotorController(port='/dev/ttyACM0')
    
    try:
        # Connect to controller
        if not controller.connect():
            print("Failed to connect to motor controller")
            return
        
        # Enable motors
        print("\n1. Enabling motors...")
        response = controller.enable_motors(True)
        print(f"Response: {response}")
        time.sleep(0.5)
        
        # Test right motor
        print("\n2. Testing right motor at 10 RPM...")
        response = controller.set_rpm("R", 10.0)
        print(f"Response: {response}")
        time.sleep(3)
        
        # Test left motor
        print("\n3. Testing left motor at -5 RPM (reverse)...")
        response = controller.set_rpm("L", -5.0)
        print(f"Response: {response}")
        time.sleep(3)
        
        # Test both motors
        print("\n4. Testing both motors...")
        response = controller.set_both_rpm(15.0, 15.0)
        print(f"Response: {response}")
        time.sleep(3)
        
        # Get status
        print("\n5. Getting status...")
        status = controller.get_status()
        if status:
            print(f"Status: {json.dumps(status, indent=2)}")
        
        # Stop motors
        print("\n6. Stopping all motors...")
        response = controller.stop_all()
        print(f"Response: {response}")
        
        # Disable motors
        print("\n7. Disabling motors...")
        response = controller.enable_motors(False)
        print(f"Response: {response}")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        # Always disconnect cleanly
        controller.disconnect()


if __name__ == "__main__":
    main()