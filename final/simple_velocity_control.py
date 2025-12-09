#!/usr/bin/env python3
"""
PID Velocity Controller

Tracks the red puck with the green paddle using PID control.
Sends velocity commands every 25ms based on pixel error.

PID Control:
    Vx = P*Ex + I*integral(Ex) + D*d(Ex)/dt
    Vy = P*Ey + I*integral(Ey) + D*d(Ey)/dt

Features:
    - PID control for smooth, accurate tracking
    - Integral term eliminates steady-state error (paddle reaches puck)
    - Derivative term reduces overshoot and oscillation
    - Boundary repulsion: smooth velocity reduction near workspace edges
    - 90-degree coordinate transform between camera and robot

Usage:
    python3 simple_velocity_control.py
"""

import serial
import json
import time
import threading
import numpy as np
import cv2
import pyzed.sl as sl


# ===================== CONFIGURATION =====================

# Serial port for motor controller (will try these in order)
MOTOR_PORTS = ["/dev/ttyACM1", "/dev/ttyACM0"]
BAUDRATE = 115200

# Control loop timing
CONTROL_INTERVAL_MS = 25  # Send commands every 25ms (40Hz)

# ===== PID CONTROLLER GAINS =====
# P (Proportional): responds to current error - main tracking force
# I (Integral): accumulates past error - eliminates steady-state error
# D (Derivative): responds to rate of change - smooths direction changes

P_GAIN = 100.0     # Proportional gain - higher = faster response
I_GAIN = 0.01    # Integral gain - lower to prevent overshoot
D_GAIN = 30.5     # Derivative gain - higher = smoother direction changes

# Integral windup limits (prevents runaway accumulation)
I_MAX = 100.0    # Max integral contribution (mm/s)

# Maximum velocity (mm/s) - from Motor_Control.cpp MAX_VELOCITY
MAX_VELOCITY = 600.0

# Minimum velocity to overcome friction (mm/s)
MIN_VELOCITY = 20.0

# Velocity smoothing (0-1): blend new velocity with previous
# 0 = no smoothing (instant), 1 = max smoothing (very slow)
# 0.3 means: new_vel = 0.7 * calculated + 0.3 * previous
VELOCITY_SMOOTHING = 0.3

# ===== COORDINATE MAPPING (set via calibration) =====
# These define how camera pixel errors map to motor velocities.
# Run with --calibrate to determine correct values for your setup.
#
# Format: (camera_axis, sign)
#   camera_axis: 'x' or 'y' - which camera axis this motor responds to
#   sign: 1 or -1 - direction (1 = same direction, -1 = inverted)
#
# Example: MOTOR_X_MAP = ('y', -1) means:
#   motor_vx = -1 * camera_error_y

MOTOR_X_MAP = ('y', 1)  # Motor X controlled by camera Y axis, inverted
MOTOR_Y_MAP = ('x', -1)  # Motor Y controlled by camera X axis, inverted

# ===== BOUNDARY HANDLING =====
# Boundary margin (in pixels from edge)
# When paddle is within this margin of an edge, repulsion kicks in
BOUNDARY_MARGIN_PX = 60

# Boundary repulsion strength (0-1 scale)
# At the edge, this fraction of MAX_VELOCITY is applied as repulsion
BOUNDARY_REPULSION_STRENGTH = 0.5

# Camera workspace boundary (pixels) - from hockey_defense_controller.py
# This defines the robot's reachable area in the camera frame
CAMERA_BOUNDARY = (145, 95, 620, 610)  # (x_min, y_min, x_max, y_max)

# HSV color ranges for detection
# Red puck (wraps around hue 0/180)
RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 100])
RED_UPPER2 = np.array([180, 255, 255])

# Green paddle
GREEN_LOWER = np.array([35, 100, 100])
GREEN_UPPER = np.array([85, 255, 255])

MIN_AREA = 80  # Minimum contour area to detect


# ===================== MOTOR CONTROLLER =====================

class MotorController:
    """Simple serial interface to the RP2350 motor controller."""
    
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connected = False
    
    def connect(self):
        """Open serial connection."""
        try:
            print(f"Connecting to {self.port}...")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            
            # Arduino resets when serial opens - must wait for it to boot
            print("Waiting for Arduino to reset...")
            time.sleep(2.0)
            
            # Clear any startup messages
            while self.ser.in_waiting:
                self.ser.readline()
            
            # Test connection
            if self.ping():
                self.connected = True
                print(f"Connected to motor controller on {self.port}")
                return True
            else:
                print("Motor controller not responding")
                return False
                
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.stop()
            self.ser.close()
            print("Disconnected from motor controller")
    
    def send_cmd(self, cmd_dict, timeout=1.0):
        """Send JSON command and get response."""
        if not self.ser or not self.ser.is_open:
            return None
        
        try:
            cmd_json = json.dumps(cmd_dict) + '\n'
            self.ser.write(cmd_json.encode('utf-8'))
            self.ser.flush()
            
            # Read response
            start = time.time()
            while (time.time() - start) < timeout:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            resp = json.loads(line)
                            # Skip debug messages, wait for actual response
                            if resp.get('debug') == 'motion':
                                continue
                            return resp
                        except json.JSONDecodeError:
                            pass
                time.sleep(0.005)
            return None
            
        except Exception as e:
            print(f"Command error: {e}")
            return None
    
    def ping(self):
        """Test connection."""
        resp = self.send_cmd({"cmd": "ping"})
        return resp is not None and resp.get("status") == "ok"
    
    def enable(self):
        """Enable motors."""
        resp = self.send_cmd({"cmd": "enable", "state": True})
        if resp and resp.get("status") == "ok":
            print("Motors enabled")
            return True
        return False
    
    def disable(self):
        """Disable motors."""
        self.send_cmd({"cmd": "enable", "state": False})
        print("Motors disabled")
    
    def stop(self):
        """Stop all movement."""
        self.send_cmd({"cmd": "stop"})
    
    def velocity(self, vx, vy):
        """
        Send velocity command (non-blocking, no response wait).
        
        Args:
            vx: X velocity in mm/s (positive = right)
            vy: Y velocity in mm/s (positive = up/away from goal)
        """
        if not self.ser or not self.ser.is_open:
            return
        
        # Clamp to max velocity
        vx = max(-MAX_VELOCITY, min(MAX_VELOCITY, vx))
        vy = max(-MAX_VELOCITY, min(MAX_VELOCITY, vy))
        
        try:
            # Drain any incoming data to prevent buffer overflow
            while self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
            
            # Send without waiting for response - keeps Arduino serial timeout happy
            cmd = json.dumps({"cmd": "velocity", "x_vel": vx, "y_vel": vy}) + '\n'
            self.ser.write(cmd.encode('utf-8'))
            self.ser.flush()
        except:
            pass
    
    def set_workspace(self, x_min=0, x_max=70, y_min=0, y_max=35):
        """Set workspace limits (if not homed)."""
        resp = self.send_cmd({
            "cmd": "set_workspace",
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max
        })
        if resp and resp.get("status") == "ok":
            print(f"Workspace set: X[{x_min}-{x_max}] Y[{y_min}-{y_max}] mm")
            return True
        return False


# ===================== CAMERA TRACKER =====================

class SimpleTracker:
    """Simplified puck and paddle tracker using ZED camera."""
    
    def __init__(self):
        self.zed = None
        self.running = False
        self.thread = None
        
        # Latest positions (in pixels, within CAMERA_BOUNDARY)
        self.puck_pos = None      # (x, y) of red puck
        self.paddle_pos = None    # (x, y) of green paddle
        self.lock = threading.Lock()
        
        # For display
        self.latest_frame = None
        self.frame_lock = threading.Lock()
    
    def start(self):
        """Initialize ZED camera and start tracking thread."""
        print("Starting ZED camera...")
        self.zed = sl.Camera()
        
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60
        init_params.depth_mode = sl.DEPTH_MODE.NONE
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to open ZED camera: {err}")
            return False
        
        print("ZED camera started")
        
        self.running = True
        self.thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """Stop tracking and close camera."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.zed:
            self.zed.close()
            print("ZED camera closed")
    
    def _detect_red_puck(self, hsv):
        """Detect red puck, return (x, y) or None."""
        mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
        mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        best_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity > 0.5 and area > best_area:
                best_area = area
                best_cnt = cnt
        
        if best_cnt is not None:
            M = cv2.moments(best_cnt)
            if M["m00"] != 0:
                return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
        return None
    
    def _detect_green_paddle(self, hsv):
        """Detect green paddle, return (x, y) or None."""
        mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_cnt = None
        best_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity > 0.5 and area > best_area:
                best_area = area
                best_cnt = cnt
        
        if best_cnt is not None:
            M = cv2.moments(best_cnt)
            if M["m00"] != 0:
                return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
        return None
    
    def _tracking_loop(self):
        """Main tracking loop - runs in separate thread."""
        image = sl.Mat()
        runtime = sl.RuntimeParameters()
        
        while self.running:
            if self.zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(image, sl.VIEW.RIGHT)
                frame = image.get_data()[:, :, :3].copy()
                
                # Convert to HSV for color detection
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                # Detect puck and paddle
                puck = self._detect_red_puck(hsv)
                paddle = self._detect_green_paddle(hsv)
                
                # Update positions thread-safely
                with self.lock:
                    self.puck_pos = puck
                    self.paddle_pos = paddle
                
                # Draw detections on frame for display
                x1, y1, x2, y2 = CAMERA_BOUNDARY
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
                
                if puck:
                    cv2.circle(frame, puck, 15, (0, 0, 255), 3)  # Red circle
                    cv2.putText(frame, "PUCK", (puck[0] - 20, puck[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                if paddle:
                    cv2.circle(frame, paddle, 15, (0, 255, 0), 3)  # Green circle
                    cv2.putText(frame, "PADDLE", (paddle[0] - 25, paddle[1] - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Draw error line if both detected
                if puck and paddle:
                    cv2.line(frame, paddle, puck, (255, 0, 255), 2)
                    ex = puck[0] - paddle[0]
                    ey = puck[1] - paddle[1]
                    mid = ((paddle[0] + puck[0]) // 2, (paddle[1] + puck[1]) // 2)
                    cv2.putText(frame, f"E:({ex},{ey})", mid,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                
                with self.frame_lock:
                    self.latest_frame = frame
            
            time.sleep(0.001)  # Prevent CPU spin
    
    def get_positions(self):
        """Get current puck and paddle positions."""
        with self.lock:
            return self.puck_pos, self.paddle_pos
    
    def get_frame(self):
        """Get latest annotated frame for display."""
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None


# ===================== CONTROL LOOP =====================

def camera_to_motor(cam_ex, cam_ey):
    """
    Convert camera pixel errors to motor velocities using calibrated mapping.
    
    Args:
        cam_ex: Camera X error (positive = target is to the RIGHT)
        cam_ey: Camera Y error (positive = target is BELOW)
    
    Returns:
        (motor_vx, motor_vy): Motor velocities
    """
    # Apply calibrated mapping
    x_axis, x_sign = MOTOR_X_MAP
    y_axis, y_sign = MOTOR_Y_MAP
    
    motor_vx = x_sign * (cam_ex if x_axis == 'x' else cam_ey)
    motor_vy = y_sign * (cam_ex if y_axis == 'x' else cam_ey)
    
    return motor_vx, motor_vy


def compute_boundary_repulsion(paddle_x, paddle_y):
    """
    Compute velocity repulsion forces to keep paddle away from boundaries.
    Uses calibrated coordinate mapping.
    """
    x_min, y_min, x_max, y_max = CAMERA_BOUNDARY
    
    # Compute camera-space repulsion (push toward center)
    cam_rep_x = 0.0
    cam_rep_y = 0.0
    
    max_repulsion = MAX_VELOCITY * BOUNDARY_REPULSION_STRENGTH
    
    # Camera LEFT edge -> push RIGHT in camera (+cam_x)
    dist_left = paddle_x - x_min
    if dist_left < BOUNDARY_MARGIN_PX:
        strength = max(0.0, min(1.0, 1.0 - dist_left / BOUNDARY_MARGIN_PX))
        cam_rep_x += max_repulsion * strength
    
    # Camera RIGHT edge -> push LEFT in camera (-cam_x)
    dist_right = x_max - paddle_x
    if dist_right < BOUNDARY_MARGIN_PX:
        strength = max(0.0, min(1.0, 1.0 - dist_right / BOUNDARY_MARGIN_PX))
        cam_rep_x -= max_repulsion * strength
    
    # Camera TOP edge -> push DOWN in camera (+cam_y)
    dist_top = paddle_y - y_min
    if dist_top < BOUNDARY_MARGIN_PX:
        strength = max(0.0, min(1.0, 1.0 - dist_top / BOUNDARY_MARGIN_PX))
        cam_rep_y += max_repulsion * strength
    
    # Camera BOTTOM edge -> push UP in camera (-cam_y)
    dist_bottom = y_max - paddle_y
    if dist_bottom < BOUNDARY_MARGIN_PX:
        strength = max(0.0, min(1.0, 1.0 - dist_bottom / BOUNDARY_MARGIN_PX))
        cam_rep_y -= max_repulsion * strength
    
    # Convert to motor coordinates
    return camera_to_motor(cam_rep_x, cam_rep_y)


def control_loop(motor, tracker):
    """
    PID control loop for puck tracking.
    
    Runs every 25ms:
    1. Get puck and paddle positions from camera
    2. Use PID control to track the puck
    3. Add boundary repulsion for smooth edge handling
    4. Send velocity command to motor controller
    """
    print("\n=== Starting PID Control Loop ===")
    print(f"Gains: P={P_GAIN}, I={I_GAIN}, D={D_GAIN}")
    print(f"Control rate: {1000/CONTROL_INTERVAL_MS:.0f} Hz")
    print(f"Boundary margin: {BOUNDARY_MARGIN_PX} px")
    print("Press Ctrl+C to stop\n")
    
    interval_sec = CONTROL_INTERVAL_MS / 1000.0
    
    # PID state variables
    integral_x = 0.0
    integral_y = 0.0
    last_error_x = 0.0
    last_error_y = 0.0
    last_time = time.time()
    
    # Velocity smoothing state
    last_vx = 0.0
    last_vy = 0.0
    
    # Track consecutive frames without paddle for recovery
    frames_without_paddle = 0
    last_known_paddle_pos = None
    
    try:
        while True:
            loop_start = time.time()
            dt = loop_start - last_time
            last_time = loop_start
            
            # Get current positions
            puck_pos, paddle_pos = tracker.get_positions()
            
            # === CASE 1: Paddle lost - emergency recovery ===
            if paddle_pos is None:
                frames_without_paddle += 1
                
                if frames_without_paddle > 5:
                    # Paddle lost for too long - try to return to center
                    x_min, y_min, x_max, y_max = CAMERA_BOUNDARY
                    
                    # Move toward center of workspace
                    if last_known_paddle_pos is not None:
                        px, py = last_known_paddle_pos
                        center_x = (x_min + x_max) / 2.0
                        center_y = (y_min + y_max) / 2.0
                        
                        # Camera error toward center
                        cam_ex = center_x - px
                        cam_ey = center_y - py
                        
                        # Convert to motor and scale to recovery speed
                        vx, vy = camera_to_motor(cam_ex, cam_ey)
                        speed = np.sqrt(vx*vx + vy*vy)
                        if speed > 0:
                            vx = vx / speed * 50.0
                            vy = vy / speed * 50.0
                        motor.velocity(vx, vy)
                        print(f"\rPADDLE LOST! Recovering... V:({vx:+.0f},{vy:+.0f})     ", end="", flush=True)
                    else:
                        motor.velocity(0, 0)
                        print(f"\rNo paddle detected - stopped                ", end="", flush=True)
                else:
                    # Brief loss - keep last velocity, might reacquire
                    print(f"\rPaddle lost ({frames_without_paddle}/5)...            ", end="", flush=True)
                
                # Skip rest of loop
                elapsed = time.time() - loop_start
                if interval_sec - elapsed > 0:
                    time.sleep(interval_sec - elapsed)
                continue
            
            # Paddle found - reset counter and save position
            frames_without_paddle = 0
            last_known_paddle_pos = paddle_pos
            
            # === CASE 2: No puck - return to center ===
            if puck_pos is None:
                # Move toward center of workspace
                x_min, y_min, x_max, y_max = CAMERA_BOUNDARY
                center_x = (x_min + x_max) / 2.0
                center_y = (y_min + y_max) / 2.0
                
                # Camera error toward center
                cam_ex = center_x - paddle_pos[0]
                cam_ey = center_y - paddle_pos[1]
                
                # Convert to motor coordinates
                ex, ey = camera_to_motor(cam_ex, cam_ey)
                
                # Simple P control to return to center
                vx = P_GAIN * ex * 0.5  # Slower when returning to center
                vy = P_GAIN * ey * 0.5
                
                # Clamp velocity
                speed = np.sqrt(vx*vx + vy*vy)
                if speed > 100:
                    vx = vx / speed * 100
                    vy = vy / speed * 100
                
                motor.velocity(vx, vy)
                print(f"\rNo puck - centering... Paddle:({paddle_pos[0]:3d},{paddle_pos[1]:3d}) "
                      f"V:({vx:+6.1f},{vy:+6.1f})    ", end="", flush=True)
                
                # Reset integral
                integral_x = 0.0
                integral_y = 0.0
            
            # === CASE 3: Normal tracking - PID control ===
            else:
                # Compute pixel error in CAMERA coordinates
                cam_ex = puck_pos[0] - paddle_pos[0]  # Positive = puck is to the RIGHT in camera
                cam_ey = puck_pos[1] - paddle_pos[1]  # Positive = puck is BELOW in camera
                
                # Convert camera error to motor error using calibrated mapping
                ex, ey = camera_to_motor(cam_ex, cam_ey)
                
                # --- Proportional term ---
                p_x = P_GAIN * ex
                p_y = P_GAIN * ey
                
                # --- Integral term (with anti-windup) ---
                integral_x += ex * dt
                integral_y += ey * dt
                
                # Clamp integral to prevent windup
                integral_x = max(-I_MAX / I_GAIN, min(I_MAX / I_GAIN, integral_x))
                integral_y = max(-I_MAX / I_GAIN, min(I_MAX / I_GAIN, integral_y))
                
                i_x = I_GAIN * integral_x
                i_y = I_GAIN * integral_y
                
                # --- Derivative term ---
                d_error_x = (ex - last_error_x) / dt if dt > 0 else 0
                d_error_y = (ey - last_error_y) / dt if dt > 0 else 0
                last_error_x = ex
                last_error_y = ey
                
                d_x = D_GAIN * d_error_x
                d_y = D_GAIN * d_error_y
                
                # --- Combine PID ---
                vx = p_x + i_x + d_x
                vy = p_y + i_y + d_y
                
                # Add minimum velocity to overcome static friction
                speed = np.sqrt(vx*vx + vy*vy)
                if 0 < speed < MIN_VELOCITY:
                    scale = MIN_VELOCITY / speed
                    vx *= scale
                    vy *= scale
                
                # Add boundary repulsion (already in motor coords)
                rep_vx, rep_vy = compute_boundary_repulsion(paddle_pos[0], paddle_pos[1])
                vx += rep_vx
                vy += rep_vy
                
                # Clamp to max velocity
                speed = np.sqrt(vx*vx + vy*vy)
                if speed > MAX_VELOCITY:
                    scale = MAX_VELOCITY / speed
                    vx *= scale
                    vy *= scale
                
                # Apply velocity smoothing for smoother direction changes
                vx = (1 - VELOCITY_SMOOTHING) * vx + VELOCITY_SMOOTHING * last_vx
                vy = (1 - VELOCITY_SMOOTHING) * vy + VELOCITY_SMOOTHING * last_vy
                last_vx = vx
                last_vy = vy
                
                # Send velocity command
                motor.velocity(vx, vy)
                
                # Status display
                speed = np.sqrt(vx*vx + vy*vy)
                print(f"\rE:({ex:+4d},{ey:+4d}) V:({vx:+6.1f},{vy:+6.1f}) spd:{speed:5.1f}", end="", flush=True)
            
            # Maintain consistent loop timing
            elapsed = time.time() - loop_start
            sleep_time = interval_sec - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\n\nStopping...")
        motor.stop()


def has_display():
    """Check if graphical display is available."""
    import os
    display = os.environ.get('DISPLAY')
    if display and display.strip():
        return True
    wayland = os.environ.get('WAYLAND_DISPLAY')
    if wayland and wayland.strip():
        return True
    return False


def display_loop(tracker):
    """Display camera view in separate thread (optional)."""
    if not has_display():
        print("No display available - skipping video window")
        print("(Run hockey_defense_controller.py and use 'view' for web streaming)")
        return
    
    print("Display started. Press 'q' to close window.")
    
    try:
        while True:
            frame = tracker.get_frame()
            if frame is not None:
                # Resize for display
                display = cv2.resize(frame, (960, 540))
                cv2.imshow("Simple Velocity Control", display)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"Display error (non-fatal): {e}")


# ===================== MAIN =====================

def calibrate(motor):
    """
    Interactive calibration to determine camera-to-motor mapping.
    Tests each motor axis and asks user which direction it moved in camera.
    """
    print("\n" + "=" * 50)
    print("CALIBRATION MODE")
    print("=" * 50)
    print("\nThis will help determine the correct coordinate mapping.")
    print("Watch the paddle in camera view and answer which way it moves.")
    print("\nMake sure paddle is visible and not at an edge.")
    input("\nPress Enter to start...")
    
    results = {}
    
    # Test Motor X
    print("\n--- Testing Motor X axis ---")
    print("Sending +100 motor_x for 1 second...")
    for _ in range(40):  # 1 second at 40Hz
        motor.velocity(100, 0)
        time.sleep(0.025)
    motor.velocity(0, 0)
    
    print("\nWhich way did the paddle move in CAMERA view?")
    print("  1 = UP")
    print("  2 = DOWN")
    print("  3 = LEFT")
    print("  4 = RIGHT")
    choice = input("Enter 1-4: ").strip()
    
    if choice == '1':
        results['motor_x'] = ('y', -1)  # +motor_x = camera up = -cam_y
    elif choice == '2':
        results['motor_x'] = ('y', 1)   # +motor_x = camera down = +cam_y
    elif choice == '3':
        results['motor_x'] = ('x', -1)  # +motor_x = camera left = -cam_x
    elif choice == '4':
        results['motor_x'] = ('x', 1)   # +motor_x = camera right = +cam_x
    else:
        print("Invalid choice, using default")
        results['motor_x'] = ('y', -1)
    
    time.sleep(0.5)
    
    # Test Motor Y
    print("\n--- Testing Motor Y axis ---")
    print("Sending +100 motor_y for 1 second...")
    for _ in range(40):
        motor.velocity(0, 100)
        time.sleep(0.025)
    motor.velocity(0, 0)
    
    print("\nWhich way did the paddle move in CAMERA view?")
    print("  1 = UP")
    print("  2 = DOWN")
    print("  3 = LEFT")
    print("  4 = RIGHT")
    choice = input("Enter 1-4: ").strip()
    
    if choice == '1':
        results['motor_y'] = ('y', -1)
    elif choice == '2':
        results['motor_y'] = ('y', 1)
    elif choice == '3':
        results['motor_y'] = ('x', -1)
    elif choice == '4':
        results['motor_y'] = ('x', 1)
    else:
        print("Invalid choice, using default")
        results['motor_y'] = ('x', -1)
    
    print("\n" + "=" * 50)
    print("CALIBRATION RESULTS")
    print("=" * 50)
    print(f"\nMOTOR_X_MAP = {results['motor_x']}")
    print(f"MOTOR_Y_MAP = {results['motor_y']}")
    print("\nUpdate these values at the top of the script!")
    print("=" * 50)
    
    return results


def main():
    import sys
    
    calibrate_mode = '--calibrate' in sys.argv
    
    print("=" * 50)
    print("PID Velocity Controller")
    print("=" * 50)
    print()
    
    if not calibrate_mode:
        print("Current mapping:")
        print(f"  MOTOR_X_MAP = {MOTOR_X_MAP}")
        print(f"  MOTOR_Y_MAP = {MOTOR_Y_MAP}")
        print("\nRun with --calibrate to recalibrate if tracking is wrong.")
        print()
    
    # Try to connect to motor controller on available ports
    motor = None
    for port in MOTOR_PORTS:
        print(f"Trying {port}...")
        motor = MotorController(port)
        if motor.connect():
            break
        motor = None
    
    if motor is None:
        print("Failed to connect to motor controller on any port")
        return
    
    # Set workspace (in case not homed)
    motor.set_workspace(x_min=0, x_max=70, y_min=0, y_max=35)
    
    # Enable motors
    if not motor.enable():
        print("Failed to enable motors")
        motor.disconnect()
        return
    
    # Initialize tracker (needed for both calibration and tracking)
    tracker = SimpleTracker()
    if not tracker.start():
        print("Failed to start camera")
        motor.disconnect()
        return
    
    # Start display thread
    display_thread = threading.Thread(target=display_loop, args=(tracker,), daemon=True)
    display_thread.start()
    
    # Wait for camera to stabilize
    print("\nWaiting for camera to stabilize...")
    time.sleep(1.0)
    
    # Run calibration if requested
    if calibrate_mode:
        try:
            calibrate(motor)
        finally:
            motor.stop()
            motor.disable()
            motor.disconnect()
            tracker.stop()
        return
    
    # Continue to tracking mode - tracker already initialized above
    
    # Run main control loop
    try:
        control_loop(motor, tracker)
    finally:
        print("\nCleaning up...")
        motor.stop()
        motor.disable()
        motor.disconnect()
        tracker.stop()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()

