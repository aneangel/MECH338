"""
Full Integration Test: Camera → Computation → Sense → ESP32

Tests the complete pipeline from puck detection to motor commands.

Usage:
    python3 test_integration.py --mode simulation  # Test without hardware
    python3 test_integration.py --mode live        # Test with real ESP32
"""

import cv2
import sys
import time
import os
import threading
from queue import Queue
from computation import TrajectoryCalculator
from camera import detect_black_objects, initialize_camera

# Try to import motor controller
try:
    from sense import MotorController
    MOTOR_CONTROLLER_AVAILABLE = True
except ImportError:
    print("Warning: MotorController not available")
    MOTOR_CONTROLLER_AVAILABLE = False
    MotorController = None


class VideoStreamThread:
    """Threaded video capture to prevent blocking"""
    
    def __init__(self, cap):
        self.cap = cap
        self.frame_queue = Queue(maxsize=2)
        self.stopped = False
        self.thread = threading.Thread(target=self._update, daemon=True)
        
    def start(self):
        self.thread.start()
        return self
    
    def _update(self):
        while not self.stopped:
            if not self.frame_queue.full():
                ret, frame = self.cap.read()
                if ret:
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except:
                            pass
                    self.frame_queue.put(frame)
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.001)
    
    def read(self):
        if not self.frame_queue.empty():
            return True, self.frame_queue.get()
        return False, None
    
    def stop(self):
        self.stopped = True
        self.thread.join()


def test_simulation_mode():
    """Test the full pipeline with simulated motor controller"""
    print("\n" + "="*70)
    print("INTEGRATION TEST - SIMULATION MODE")
    print("Testing: Camera → Computation → Motor Commands (Simulated)")
    print("="*70)
    
    # Initialize camera
    cap = initialize_camera()
    if cap is None:
        print("ERROR: Failed to initialize camera!")
        return
    
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    video_stream = VideoStreamThread(cap).start()
    time.sleep(1.0)
    
    # Initialize trajectory calculator WITHOUT motor controller
    calc = TrajectoryCalculator(motor_controller=None)
    
    print("\n[SYSTEM INITIALIZED]")
    print("Camera: READY")
    print("Computation: READY")
    print("Motor Controller: SIMULATED")
    print("\nMove a black object in camera view...")
    print("Press Ctrl+C to stop")
    print("-" * 70)
    
    frame_count = 0
    detection_count = 0
    command_count = 0
    
    start_time = time.time()
    last_frame_time = time.time()
    
    try:
        while True:
            ret, frame = video_stream.read()
            current_time = time.time()
            
            if not ret or frame is None:
                time.sleep(0.001)
                continue
            
            frame_count += 1
            frame_time = current_time - last_frame_time
            fps = 1.0 / frame_time if frame_time > 0 else 0
            last_frame_time = current_time
            
            # Step 1: CAMERA DETECTION
            detections, _ = detect_black_objects(frame)
            
            if detections:
                detection_count += 1
                largest = max(detections, key=lambda d: d['area'])
                
                print(f"\n{'='*70}")
                print(f"[FRAME {frame_count}] DETECTION #{detection_count} (FPS: {fps:.1f})")
                print(f"{'='*70}")
                
                # Step 2: COMPUTATION - Update position
                calc.update_puck_position(largest)
                
                print(f"[CAMERA] Pixel: {largest['center']}, Area: {largest['area']:.0f} px²")
                
                if calc.current_position:
                    x_mm, y_mm = calc.current_position
                    print(f"[COMPUTATION] Position: ({x_mm:.2f}, {y_mm:.2f}) mm")
                
                if calc.current_velocity:
                    vx, vy = calc.current_velocity
                    speed = (vx**2 + vy**2) ** 0.5
                    print(f"[COMPUTATION] Velocity: ({vx:.1f}, {vy:.1f}) mm/s | Speed: {speed:.1f} mm/s")
                    
                    # Predict future position
                    future = calc.predict_future_position(0.5)
                    if future:
                        print(f"[COMPUTATION] Predicted (0.5s): ({future[0]:.2f}, {future[1]:.2f}) mm")
                    
                    # Step 3: INTERCEPTION CALCULATION
                    intercept = calc.calculate_interception_point()
                    
                    if intercept:
                        command_count += 1
                        print(f"\n>>> INTERCEPTION NEEDED <<<")
                        print(f"[COMPUTATION] Intercept Point: ({intercept[0]:.2f}, {intercept[1]:.2f}) mm")
                        
                        # Calculate time to intercept
                        time_to_intercept = abs((intercept[1] - y_mm) / vy) if abs(vy) > 0.1 else 0.5
                        
                        # Step 4: GENERATE MOTOR COMMANDS
                        motor_cmds = calc.get_motor_commands(intercept, time_to_intercept)
                        
                        print(f"\n[MOTOR COMMANDS] Command #{command_count}")
                        print(f"  Target Position: ({motor_cmds['target_position'][0]:.2f}, {motor_cmds['target_position'][1]:.2f}) mm")
                        print(f"  Time to Intercept: {motor_cmds['time_to_intercept']:.3f} s")
                        print(f"  Motor A (Right): {motor_cmds['revs_right']:.3f} revs @ {motor_cmds['rpm_right']:.1f} RPM")
                        print(f"  Motor B (Left):  {motor_cmds['revs_left']:.3f} revs @ {motor_cmds['rpm_left']:.1f} RPM")
                        
                        # SIMULATE sending to ESP32
                        print(f"\n[SENSE] SIMULATED: Would send to ESP32:")
                        print(f"  set_both_rpm({motor_cmds['rpm_right']:.1f}, {motor_cmds['rpm_left']:.1f})")
                        print(f"{'='*70}")
                    else:
                        print(f"[COMPUTATION] No interception needed (puck not moving toward defense)")
            
            elif frame_count % 30 == 0:
                print(f"[Frame {frame_count}] No detection (FPS: {fps:.1f})")
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted")
    
    finally:
        video_stream.stop()
        cap.release()
        
        elapsed = time.time() - start_time
        print("\n" + "="*70)
        print("SIMULATION TEST COMPLETE")
        print("="*70)
        print(f"Runtime: {elapsed:.1f}s")
        print(f"Frames: {frame_count} ({frame_count/elapsed:.1f} FPS)")
        print(f"Detections: {detection_count}")
        print(f"Motor Commands Generated: {command_count}")
        print("="*70)


def test_live_mode():
    """Test with REAL ESP32 motor controller"""
    
    if not MOTOR_CONTROLLER_AVAILABLE:
        print("\nERROR: MotorController not available!")
        print("Make sure sense.py is in the same directory.")
        return
    
    print("\n" + "="*70)
    print("INTEGRATION TEST - LIVE MODE")
    print("Testing: Camera → Computation → Sense → ESP32 (REAL HARDWARE)")
    print("="*70)
    
    # Initialize camera
    cap = initialize_camera()
    if cap is None:
        print("ERROR: Failed to initialize camera!")
        return
    
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    video_stream = VideoStreamThread(cap).start()
    time.sleep(1.0)
    
    # Initialize motor controller
    print("\n[CONNECTING TO ESP32]")
    motor_controller = MotorController(port='/dev/ttyACM0', baudrate=115200)
    
    if not motor_controller.connect():
        print("ERROR: Failed to connect to ESP32!")
        print("Make sure:")
        print("  1. ESP32 is connected via USB")
        print("  2. Port is /dev/ttyACM0 (or update in sense.py)")
        print("  3. ESP32 code is uploaded and running")
        cap.release()
        return
    
    print("[ESP32 CONNECTED]")
    
    # Test motor controller
    print("\n[TESTING ESP32 COMMUNICATION]")
    response = motor_controller.ping()
    if response:
        print(f"Ping response: {response}")
    
    # Enable motors
    print("[ENABLING MOTORS]")
    motor_controller.enable_motors(True, "both")
    
    # Initialize trajectory calculator WITH motor controller
    calc = TrajectoryCalculator(motor_controller=motor_controller)
    
    print("\n[SYSTEM INITIALIZED]")
    print("Camera: READY")
    print("Computation: READY")
    print("Motor Controller: CONNECTED")
    print("ESP32: READY")
    print("\nMove a black object in camera view...")
    print("Press Ctrl+C to stop")
    print("-" * 70)
    
    frame_count = 0
    detection_count = 0
    command_count = 0
    
    start_time = time.time()
    last_frame_time = time.time()
    
    try:
        while True:
            ret, frame = video_stream.read()
            current_time = time.time()
            
            if not ret or frame is None:
                time.sleep(0.001)
                continue
            
            frame_count += 1
            frame_time = current_time - last_frame_time
            fps = 1.0 / frame_time if frame_time > 0 else 0
            last_frame_time = current_time
            
            # Step 1: CAMERA DETECTION
            detections, _ = detect_black_objects(frame)
            
            if detections:
                detection_count += 1
                largest = max(detections, key=lambda d: d['area'])
                
                print(f"\n{'='*70}")
                print(f"[FRAME {frame_count}] DETECTION #{detection_count} (FPS: {fps:.1f})")
                print(f"{'='*70}")
                
                # Step 2: COMPUTATION - Process detection
                result = calc.process_detection(largest)
                
                print(f"[CAMERA] Pixel: {largest['center']}, Area: {largest['area']:.0f} px²")
                
                if result:
                    command_count += 1
                    print(f"[COMPUTATION] Position: ({result['puck_position'][0]:.2f}, {result['puck_position'][1]:.2f}) mm")
                    print(f"[COMPUTATION] Velocity: ({result['puck_velocity'][0]:.1f}, {result['puck_velocity'][1]:.1f}) mm/s")
                    print(f"[COMPUTATION] Intercept: ({result['intercept_point'][0]:.2f}, {result['intercept_point'][1]:.2f}) mm")
                    print(f"[COMPUTATION] Time: {result['time_to_intercept']:.3f} s")
                    
                    # Step 3: MOTOR COMMAND SENT
                    if result['command_sent']:
                        print(f"\n>>> COMMAND #{command_count} SENT TO ESP32 <<<")
                        print(f"[SENSE] Motors commanded to intercept puck!")
                        print(f"{'='*70}")
                    else:
                        print(f"[SENSE] ERROR: Failed to send command to ESP32")
                else:
                    if calc.current_position:
                        print(f"[COMPUTATION] Position: ({calc.current_position[0]:.2f}, {calc.current_position[1]:.2f}) mm")
                    if calc.current_velocity:
                        print(f"[COMPUTATION] Velocity: ({calc.current_velocity[0]:.1f}, {calc.current_velocity[1]:.1f}) mm/s")
                    print(f"[COMPUTATION] No interception needed")
            
            elif frame_count % 30 == 0:
                print(f"[Frame {frame_count}] No detection (FPS: {fps:.1f})")
    
    except KeyboardInterrupt:
        print("\n\n[EMERGENCY STOP]")
        calc.stop_motors()
    
    finally:
        video_stream.stop()
        cap.release()
        motor_controller.disconnect()
        
        elapsed = time.time() - start_time
        print("\n" + "="*70)
        print("LIVE TEST COMPLETE")
        print("="*70)
        print(f"Runtime: {elapsed:.1f}s")
        print(f"Frames: {frame_count} ({frame_count/elapsed:.1f} FPS)")
        print(f"Detections: {detection_count}")
        print(f"Motor Commands Sent: {command_count}")
        print("="*70)


def test_motor_only():
    """Test motor controller independently"""
    
    if not MOTOR_CONTROLLER_AVAILABLE:
        print("\nERROR: MotorController not available!")
        return
    
    print("\n" + "="*70)
    print("MOTOR CONTROLLER TEST")
    print("="*70)
    
    controller = MotorController(port='/dev/ttyACM0')
    
    if not controller.connect():
        print("ERROR: Failed to connect to ESP32")
        return
    
    print("✓ Connected to ESP32")
    
    # Test ping
    print("\nTesting ping...")
    resp = controller.ping()
    print(f"Ping response: {resp}")
    
    # Get status
    print("\nGetting status...")
    status = controller.get_status()
    print(f"Status: {status}")
    
    # Enable motors
    print("\nEnabling motors...")
    resp = controller.enable_motors(True, "both")
    print(f"Enable response: {resp}")
    
    # Test movement
    print("\nTesting motor movement (10 RPM for 2 seconds)...")
    resp = controller.set_both_rpm(10.0, 10.0)
    print(f"Set RPM response: {resp}")
    time.sleep(2.0)
    
    # Stop
    print("\nStopping motors...")
    resp = controller.stop_all()
    print(f"Stop response: {resp}")
    
    # Disconnect
    controller.disconnect()
    print("\n✓ Motor controller test complete")
    print("="*70)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Integration test for air hockey robot')
    parser.add_argument('--mode', choices=['simulation', 'live', 'motor'],
                       default='simulation',
                       help='Test mode: simulation (no ESP32), live (with ESP32), motor (ESP32 only)')
    
    args = parser.parse_args()
    
    if args.mode == 'simulation':
        test_simulation_mode()
    elif args.mode == 'live':
        test_live_mode()
    elif args.mode == 'motor':
        test_motor_only()