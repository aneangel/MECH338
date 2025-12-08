#!/usr/bin/env python3
"""
Air Hockey Defense Controller

Integrates camera tracking with motor control to defend the goal.
- Direct ZED camera access via USB 3.0
- Web streaming for real-time view (http://<ip>:5001)
- Predicts puck trajectory with Kalman filter
- Commands paddle to intercept
- Uses RP2350 CoreXY controller with sensorless homing

Usage:
    python3 hockey_defense_controller.py

Commands (manual mode):
    right, left, up, down  - Move paddle
    stop                    - Stop movement
    view                    - Toggle web stream (http://<ip>:5001)
    home                    - Run sensorless homing
    camera_home             - Camera-guided safe homing
    localize                - Use camera to find current position (no motors)
    auto                    - Start autonomous defense
    quit                    - Exit
"""

import serial
import json
import time
import sys
import serial.tools.list_ports
import threading
from collections import deque
import numpy as np
import cv2
import pyzed.sl as sl
from flask import Flask, Response


def find_motor_controller():
    """Find XIAO RP2350 motor controller."""
    # MANUAL OVERRIDE: Set to specific port if you have multiple RP2350s
    # Uncomment one of these if auto-detect picks the wrong one:
    return "/dev/ttyACM0"
    # return "/dev/ttyACM2"
    
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == 0x2886 and port.pid == 0x58:
            return port.device
    return None


class PuckTracker:
    """Track puck directly from ZED camera via USB 3.0."""
    
    def __init__(self):
        # ZED camera
        self.zed = None
        self.runtime_params = None
        self.camera_view = sl.VIEW.RIGHT
        
        # Crop region - will be set from detected table boundary
        self.CROP_REGION = None  # (x1, y1, x2, y2) in original frame
        self.use_crop = False  # Enable cropping to zoom on table
        self.crop_padding = 20  # Extra pixels around detected table for context
        
        # Manual workspace boundary (robot's defense zone - left half of table)
        # Set to None to use auto-detection, or (x1, y1, x2, y2) for fixed boundary
        # These values are for the robot's reachable workspace on the table
        self.MANUAL_BOUNDARY = (145, 95, 620, 610)  # Left half of table (x1, y1, x2, y2)
        self.use_manual_boundary = True  # Set to True to use fixed boundary
        
        # HSV color ranges for red puck
        self.RED_LOWER1 = np.array([0, 100, 100])
        self.RED_UPPER1 = np.array([10, 255, 255])
        self.RED_LOWER2 = np.array([160, 100, 100])
        self.RED_UPPER2 = np.array([180, 255, 255])
        self.MIN_PUCK_AREA = 80
        
        # HSV color ranges for green paddle (end-effector)
        self.GREEN_LOWER = np.array([35, 100, 100])
        self.GREEN_UPPER = np.array([85, 255, 255])
        self.MIN_PADDLE_AREA = 80
        
        # HSV color ranges for table surface detection
        # The table is tan/cream/white - wide range to handle lighting variations
        # Low saturation catches the cream/white areas, higher catches tan
        self.TABLE_LOWER = np.array([10, 20, 80])   # Hue 10-45, low-med sat, med-high value
        self.TABLE_UPPER = np.array([45, 200, 255])
        
        # Glare threshold - pixels above this brightness with low saturation are glare
        self.GLARE_VALUE_THRESHOLD = 240  # Very bright
        self.GLARE_SAT_THRESHOLD = 30     # Low saturation = white/glare
        
        # Kalman filter for puck tracking
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1,0,0,0], [0,1,0,0]], np.float32)
        self.kf.transitionMatrix = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 10
        
        # Kalman filter for paddle (green circle) tracking
        self.kf_paddle = cv2.KalmanFilter(4, 2)
        self.kf_paddle.measurementMatrix = np.array([[1,0,0,0], [0,1,0,0]], np.float32)
        self.kf_paddle.transitionMatrix = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]], np.float32)
        self.kf_paddle.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kf_paddle.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5
        
        # Tracking state
        self.last_puck_pos = None
        self.last_puck_vel = None
        self.puck_found = False
        self.frames_without_puck = 0
        self.puck_lost_threshold = 30  # Stop if puck missing for 30 frames (~0.5s at 60fps)
        
        self.last_paddle_pos = None
        self.paddle_found = False
        self.goal_x = 0.0
        
        # Virtual boundary detection (brown table with black edges)
        self.virtual_boundary = None  # Will be (x_min, y_min, x_max, y_max) in camera pixels
        self.boundary_margin = 15  # pixels safety margin from detected edge
        
        # Raw detection positions for drawing (not Kalman filtered)
        self.raw_puck_pos = None
        self.raw_paddle_pos = None
        
        # Display settings
        self.show_preview = False
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Flask streaming
        self.flask_app = None
        self.flask_thread = None
        self.stream_port = 5001  # Different port from stream_camera.py
        
        self.running = False
        self.thread = None
        
    def initialize_camera(self):
        """Initialize ZED camera."""
        print("Initializing ZED camera...")
        self.zed = sl.Camera()
        
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60
        init_params.depth_mode = sl.DEPTH_MODE.NONE  # We only need RGB
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"✗ Camera initialization failed: {err}")
            return False
        
        self.runtime_params = sl.RuntimeParameters()
        print("✓ ZED camera initialized")
        return True
        
    def start(self):
        """Start tracking thread."""
        if not self.initialize_camera():
            return False
        
        image = sl.Mat()
        frame_w, frame_h = 1280, 720  # Default
        
        # Get frame dimensions
        if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(image, sl.VIEW.RIGHT)
            frame = image.get_data()[:, :, :3]
            frame_h, frame_w = frame.shape[:2]
            print(f"Frame size: {frame_w}x{frame_h}")
        
        # === USE MANUAL BOUNDARY (robot's workspace on left half of table) ===
        if self.use_manual_boundary and self.MANUAL_BOUNDARY:
            x_min, y_min, x_max, y_max = self.MANUAL_BOUNDARY
            self.virtual_boundary = (x_min, y_min, x_max, y_max)
            print(f"Using manual workspace boundary: ({x_min}, {y_min}) to ({x_max}, {y_max})")
            print(f"  Workspace size: {x_max-x_min}x{y_max-y_min} pixels")
        else:
            # === AUTO-DETECT TABLE BOUNDARY ===
            print("Detecting table boundary...")
            
            for attempt in range(10):
                if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                    self.zed.retrieve_image(image, sl.VIEW.RIGHT)
                    frame = image.get_data()[:, :, :3]
                    
                    boundary = self.detect_table_boundary(frame)
                    if boundary:
                        x_min, y_min, x_max, y_max = boundary
                        
                        # Set crop region slightly larger than boundary for context
                        crop_x1 = max(0, x_min - self.crop_padding - self.boundary_margin)
                        crop_y1 = max(0, y_min - self.crop_padding - self.boundary_margin)
                        crop_x2 = min(frame_w, x_max + self.crop_padding + self.boundary_margin)
                        crop_y2 = min(frame_h, y_max + self.crop_padding + self.boundary_margin)
                        self.CROP_REGION = (crop_x1, crop_y1, crop_x2, crop_y2)
                        
                        # Virtual boundary in frame coordinates (no cropping)
                        self.virtual_boundary = (x_min, y_min, x_max, y_max)
                        
                        print(f"Table detected: ({x_min}, {y_min}) to ({x_max}, {y_max})")
                        break
                    else:
                        print(f"  Attempt {attempt + 1}/10: No boundary detected")
                time.sleep(0.1)
            
            if self.virtual_boundary is None:
                print("Warning: Could not detect table boundary. Using full frame.")
                self.virtual_boundary = (self.boundary_margin, self.boundary_margin, 
                                        frame_w - self.boundary_margin, frame_h - self.boundary_margin)
        
        # Start tracking thread
        self.running = True
        self.thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """Stop tracking thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.zed:
            self.zed.close()
            print("ZED camera closed")
    
    def _detect_puck(self, frame_hsv):
        """Detect red puck in frame, filtering out glare."""
        mask1 = cv2.inRange(frame_hsv, self.RED_LOWER1, self.RED_UPPER1)
        mask2 = cv2.inRange(frame_hsv, self.RED_LOWER2, self.RED_UPPER2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Filter out glare (very bright, low saturation areas)
        glare_mask = cv2.inRange(frame_hsv, 
                                  np.array([0, 0, self.GLARE_VALUE_THRESHOLD]),
                                  np.array([180, self.GLARE_SAT_THRESHOLD, 255]))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(glare_mask))
        
        # Clean up mask
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_contour = None
        best_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_PUCK_AREA:
                continue
            
            # Check circularity
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            if circularity > 0.6 and area > best_area:
                best_area = area
                best_contour = cnt
        
        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
        
        return None
    
    def _detect_paddle(self, frame_hsv):
        """Detect green paddle (end-effector) in frame, filtering out glare."""
        mask = cv2.inRange(frame_hsv, self.GREEN_LOWER, self.GREEN_UPPER)
        
        # Filter out glare
        glare_mask = cv2.inRange(frame_hsv, 
                                  np.array([0, 0, self.GLARE_VALUE_THRESHOLD]),
                                  np.array([180, self.GLARE_SAT_THRESHOLD, 255]))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(glare_mask))
        
        # Clean up mask
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_contour = None
        best_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_PADDLE_AREA:
                continue
            
            # Check circularity
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            if circularity > 0.6 and area > best_area:
                best_area = area
                best_contour = cnt
        
        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
        
        return None
    
    def detect_table_boundary(self, frame):
        """Detect table playing surface bounded by black edges.
        
        Strategy:
        1. Find the black rectangular border around the table
        2. The playing surface is inside that border
        3. Erode slightly to get the actual playing area
        
        Args:
            frame: BGR image from camera
            
        Returns:
            tuple: (x_min, y_min, x_max, y_max) boundary in pixels, or None
        """
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # === METHOD 1: Find black border, then get inner area ===
        # The table has a distinct black border - find it first
        
        # Detect the tan/cream playing surface (not black, not too bright)
        # Table surface: tan/cream color, medium brightness
        table_mask = cv2.inRange(hsv, 
                                  np.array([10, 15, 100]),   # Low: tan hue, some sat, medium value
                                  np.array([50, 150, 245]))  # High: yellow-ish, not too saturated, not glare
        
        # Remove glare spots
        glare = cv2.inRange(hsv, np.array([0, 0, 245]), np.array([180, 40, 255]))
        table_mask = cv2.bitwise_and(table_mask, cv2.bitwise_not(glare))
        
        # Clean up - close small gaps, remove noise
        kernel = np.ones((7, 7), np.uint8)
        table_mask = cv2.morphologyEx(table_mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        table_mask = cv2.morphologyEx(table_mask, cv2.MORPH_OPEN, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(table_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            print("  No table surface found")
            return self._detect_by_black_border(frame)
        
        # Find the largest contour that's reasonably rectangular
        min_area = (w * h) * 0.2  # Table should be at least 20% of frame
        best_contour = None
        best_rect_score = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            
            # Approximate contour to polygon
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            # Get minimum area rectangle
            rect = cv2.minAreaRect(cnt)
            rect_area = rect[1][0] * rect[1][1]
            
            if rect_area == 0:
                continue
            
            # Rectangularity score
            rectangularity = area / rect_area
            
            # Prefer contours that are large and rectangular
            score = area * rectangularity
            if score > best_rect_score and rectangularity > 0.75:
                best_rect_score = score
                best_contour = cnt
        
        if best_contour is None:
            print("  No rectangular table surface found")
            return self._detect_by_black_border(frame)
        
        # Use minimum area rectangle for cleaner boundaries
        rect = cv2.minAreaRect(best_contour)
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        
        # Get axis-aligned bounding box from the rotated rect
        x_coords = box[:, 0]
        y_coords = box[:, 1]
        
        x_min = int(np.min(x_coords))
        x_max = int(np.max(x_coords))
        y_min = int(np.min(y_coords))
        y_max = int(np.max(y_coords))
        
        # Clamp to frame
        x_min = max(0, x_min)
        y_min = max(0, y_min)
        x_max = min(w, x_max)
        y_max = min(h, y_max)
        
        # Apply inward margin (the detected surface might include a bit of the black edge)
        margin = self.boundary_margin + 5  # Extra 5px inward
        x_min += margin
        y_min += margin
        x_max -= margin
        y_max -= margin
        
        # Sanity check
        if (x_max - x_min) < 200 or (y_max - y_min) < 100:
            print(f"  Boundary too small: {x_max-x_min}x{y_max-y_min}")
            return self._detect_by_black_border(frame)
        
        print(f"  Table surface: {x_max-x_min}x{y_max-y_min} px")
        return (x_min, y_min, x_max, y_max)
    
    def _detect_by_black_border(self, frame):
        """Fallback: detect the black rectangular border around table."""
        print("  Trying black border detection...")
        
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect black areas (the border)
        black_mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 60]))
        
        # Clean up
        kernel = np.ones((5, 5), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        
        # Find contours in the black mask
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return self._edge_fallback(frame)
        
        # Find the largest black rectangular shape (the table border)
        min_area = (w * h) * 0.15
        best_contour = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area and area > min_area:
                # Check if it's rectangular
                rect = cv2.minAreaRect(cnt)
                rect_area = rect[1][0] * rect[1][1]
                if rect_area > 0 and area / rect_area > 0.5:
                    max_area = area
                    best_contour = cnt
        
        if best_contour is None:
            return self._edge_fallback(frame)
        
        # Get bounding rect and shrink inward (we want the inside)
        x, y, w_rect, h_rect = cv2.boundingRect(best_contour)
        
        # The playing surface is inside the black border
        # Estimate border thickness and subtract
        border_thickness = 25  # Approximate black border width
        
        x_min = x + border_thickness
        y_min = y + border_thickness  
        x_max = x + w_rect - border_thickness
        y_max = y + h_rect - border_thickness
        
        if (x_max - x_min) > 200 and (y_max - y_min) > 100:
            print(f"  Found via black border: {x_max-x_min}x{y_max-y_min} px")
            return (x_min, y_min, x_max, y_max)
        
        return self._edge_fallback(frame)
    
    def _edge_fallback(self, frame):
        """Last resort: edge detection."""
        print("  Edge detection fallback...")
        
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=2)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        min_area = (w * h) * 0.1
        best_rect = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area and area > min_area:
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                if w_rect > h_rect * 0.8 and w_rect > w * 0.3:
                    max_area = area
                    best_rect = (x + 20, y + 20, x + w_rect - 20, y + h_rect - 20)
        
        if best_rect:
            print(f"  Edge fallback: {best_rect[2]-best_rect[0]}x{best_rect[3]-best_rect[1]} px")
            return best_rect
        
        return None
    
    def is_near_boundary(self, paddle_pos, threshold=10):
        """Check if paddle is approaching virtual boundary.
        
        Args:
            paddle_pos: (x, y) position in camera pixels
            threshold: Distance threshold in pixels
            
        Returns:
            tuple: (near_boundary, safe_position) where safe_position is clamped coords
        """
        if self.virtual_boundary is None or paddle_pos is None:
            return False, paddle_pos
        
        x, y = paddle_pos
        x_min, y_min, x_max, y_max = self.virtual_boundary
        
        # Check if near any boundary
        near_left = (x - x_min) < threshold
        near_right = (x_max - x) < threshold
        near_top = (y - y_min) < threshold
        near_bottom = (y_max - y) < threshold
        
        near_boundary = near_left or near_right or near_top or near_bottom
        
        # Clamp to safe position
        safe_x = max(x_min + threshold, min(x_max - threshold, x))
        safe_y = max(y_min + threshold, min(y_max - threshold, y))
        
        return near_boundary, (safe_x, safe_y)
    
    def _tracking_loop(self):
        """Background thread for direct camera tracking."""
        image = sl.Mat()
        
        while self.running:
            try:
                if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                    self.zed.retrieve_image(image, sl.VIEW.RIGHT)
                    frame = image.get_data()[:, :, :3]  # BGRA -> BGR
                    
                    # Use full frame or crop
                    if self.use_crop and self.CROP_REGION:
                        x1, y1, x2, y2 = self.CROP_REGION
                        cropped = frame[y1:y2, x1:x2]
                    else:
                        cropped = frame  # Use full frame
                    
                    # Convert to HSV
                    frame_hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
                    
                    # Detect puck
                    detection = self._detect_puck(frame_hsv)
                    
                    if detection is not None:
                        # Update Kalman filter
                        measurement = np.array([[np.float32(detection[0])],
                                              [np.float32(detection[1])]])
                        self.kf.correct(measurement)
                        self.puck_found = True
                        self.frames_without_puck = 0  # Reset counter
                        self.raw_puck_pos = detection  # Store raw detection for drawing
                    else:
                        self.puck_found = False
                        self.frames_without_puck += 1
                        self.raw_puck_pos = None
                    
                    # Always predict (coast when not detected)
                    prediction = self.kf.predict()
                    
                    # Extract position and velocity
                    self.last_puck_pos = (float(prediction[0, 0]), float(prediction[1, 0]))
                    self.last_puck_vel = (float(prediction[2, 0]), float(prediction[3, 0]))
                    
                    # Detect paddle (green circle)
                    paddle_detection = self._detect_paddle(frame_hsv)
                    
                    if paddle_detection is not None:
                        # Update Kalman filter for paddle
                        paddle_measurement = np.array([[np.float32(paddle_detection[0])],
                                                      [np.float32(paddle_detection[1])]])
                        self.kf_paddle.correct(paddle_measurement)
                        self.paddle_found = True
                        self.raw_paddle_pos = paddle_detection  # Store raw detection for drawing
                    else:
                        self.paddle_found = False
                        self.raw_paddle_pos = None
                    
                    # Always predict paddle position
                    paddle_prediction = self.kf_paddle.predict()
                    
                    # Extract paddle position
                    self.last_paddle_pos = (float(paddle_prediction[0, 0]), float(paddle_prediction[1, 0]))
                    
                    # Always create annotated frame for streaming
                    preview_frame = cropped.copy()
                    
                    # Draw virtual boundary
                    if self.virtual_boundary:
                        x_min, y_min, x_max, y_max = self.virtual_boundary
                        # Convert to int for OpenCV
                        x_min, y_min, x_max, y_max = int(x_min), int(y_min), int(x_max), int(y_max)
                        cv2.rectangle(preview_frame, (x_min, y_min), (x_max, y_max), (255, 255, 0), 2)
                        cv2.putText(preview_frame, "Virtual Boundary", (x_min + 5, y_min + 20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # Draw puck detection
                    if detection:
                        cv2.circle(preview_frame, detection, 10, (0, 0, 255), 2)
                        cv2.putText(preview_frame, "PUCK", (detection[0] + 15, detection[1]), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    # Draw paddle detection
                    if paddle_detection:
                        cv2.circle(preview_frame, paddle_detection, 10, (0, 255, 0), 2)
                        cv2.putText(preview_frame, "PADDLE", (paddle_detection[0] + 15, paddle_detection[1]), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Draw intercept line between paddle and puck when both detected
                    if detection and paddle_detection:
                        cv2.line(preview_frame, paddle_detection, detection, (255, 0, 255), 2)
                        # Calculate distance for display
                        dx = detection[0] - paddle_detection[0]
                        dy = detection[1] - paddle_detection[1]
                        dist = np.sqrt(dx*dx + dy*dy)
                        mid_x = (detection[0] + paddle_detection[0]) // 2
                        mid_y = (detection[1] + paddle_detection[1]) // 2
                        cv2.putText(preview_frame, f"{dist:.0f}px", (mid_x + 5, mid_y - 5),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    
                    # Draw predicted puck trajectory if moving
                    if self.last_puck_vel and self.puck_found:
                        vx, vy = self.last_puck_vel
                        speed = np.sqrt(vx*vx + vy*vy)
                        if speed > 2.0:  # Only draw if moving fast enough
                            px, py = int(self.last_puck_pos[0]), int(self.last_puck_pos[1])
                            # Predict 10 steps into future
                            future_pts = []
                            for i in range(1, 11):
                                fx = int(px + vx * i * 2)
                                fy = int(py + vy * i * 2)
                                future_pts.append((fx, fy))
                            if len(future_pts) > 1:
                                cv2.polylines(preview_frame, [np.array(future_pts)], False, (0, 0, 255), 2)
                    
                    # Add detection status text
                    in_play = self.is_puck_in_play()
                    status_text = f"Puck: {'IN PLAY' if in_play else 'OUT OF PLAY'}  Paddle: {'FOUND' if self.paddle_found else 'LOST'}"
                    status_color = (0, 255, 0) if in_play else (0, 0, 255)
                    cv2.putText(preview_frame, status_text, (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                    
                    # Add frame size info
                    h, w = preview_frame.shape[:2]
                    cv2.putText(preview_frame, f"Frame: {w}x{h}", (10, h - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    with self.frame_lock:
                        self.latest_frame = preview_frame
                
            except Exception as e:
                print(f"Tracking error: {e}")
                time.sleep(0.05)
            
            time.sleep(0.001)  # Minimal sleep for high-speed tracking
    
    def is_puck_in_play(self):
        """Check if puck is currently in play (being tracked).
        
        Returns:
            bool: True if puck is detected, False if lost/goal scored
        """
        return self.frames_without_puck < self.puck_lost_threshold
    
    def get_puck_state(self):
        """Get latest puck position and velocity.
        
        Returns:
            tuple: ((x, y), (vx, vy)) or (None, None) if not visible
        """
        if self.last_puck_pos is None:
            return None, None
        return self.last_puck_pos, self.last_puck_vel
    
    def get_paddle_position(self):
        """Get latest paddle position from camera.
        
        Returns:
            tuple: (x, y) in camera pixels, or None if not visible
        """
        if self.last_paddle_pos is None or not self.paddle_found:
            return None
        return self.last_paddle_pos
    
    def set_workspace_limits(self, x_min, x_max, y_min, y_max):
        """Set workspace dimensions from motor controller after homing.
        
        This allows accurate camera-to-workspace coordinate conversion.
        
        Args:
            x_min, x_max: X axis limits in mm
            y_min, y_max: Y axis limits in mm
        """
        self.workspace_x_min = x_min
        self.workspace_x_max = x_max
        self.workspace_y_min = y_min
        self.workspace_y_max = y_max
        print(f"Workspace set: X[{x_min:.1f}-{x_max:.1f}] Y[{y_min:.1f}-{y_max:.1f}] mm")
    
    def camera_to_workspace(self, cam_x, cam_y):
        """Convert camera pixel coordinates to workspace mm coordinates.
        
        The camera's virtual boundary (table surface) maps to the motor's
        workspace limits from sensorless homing (ground truth).
        
        COORDINATE SYSTEMS:
        - Camera: (0,0) top-left, X increases right, Y increases DOWN
        - Motor:  (0,0) bottom-left, X increases right, Y increases UP
        - So Y axis is INVERTED between camera and motor
        
        Args:
            cam_x: Camera X coordinate (pixels, in cropped/boundary frame)
            cam_y: Camera Y coordinate (pixels, in cropped/boundary frame)
        
        Returns:
            tuple: (x_mm, y_mm) in workspace coordinates
        """
        # Get pixel frame dimensions from virtual boundary
        if self.virtual_boundary:
            x_min, y_min, x_max, y_max = self.virtual_boundary
            frame_width = x_max - x_min
            frame_height = y_max - y_min
            # Normalize to boundary (0 to frame_width/height)
            cam_x = cam_x - x_min
            cam_y = cam_y - y_min
        else:
            frame_width = 1280
            frame_height = 720
        
        # Use workspace limits from sensorless homing (GROUND TRUTH - no defaults!)
        ws_x_min = getattr(self, 'workspace_x_min', None)
        ws_x_max = getattr(self, 'workspace_x_max', None)
        ws_y_min = getattr(self, 'workspace_y_min', None)
        ws_y_max = getattr(self, 'workspace_y_max', None)
        
        # If not homed, return None (can't map coordinates)
        if ws_x_max is None or ws_y_max is None:
            return None, None
        
        ws_x_range = ws_x_max - ws_x_min
        ws_y_range = ws_y_max - ws_y_min
        
        # Map camera pixels to workspace mm
        # X: camera left (0) = motor X_min, camera right = motor X_max
        workspace_x = ws_x_min + (cam_x / frame_width) * ws_x_range
        
        # Y: INVERTED - camera top (0) = motor Y_max, camera bottom = motor Y_min
        workspace_y = ws_y_max - (cam_y / frame_height) * ws_y_range
        
        # Clamp to workspace limits
        workspace_x = max(ws_x_min, min(ws_x_max, workspace_x))
        workspace_y = max(ws_y_min, min(ws_y_max, workspace_y))
        
        return workspace_x, workspace_y
    
    def toggle_preview(self):
        """Toggle camera preview - starts/stops Flask web streaming."""
        self.show_preview = not self.show_preview
        if self.show_preview:
            self.start_stream_server()
        else:
            self.stop_stream_server()
    
    def start_stream_server(self):
        """Start Flask server for real-time video streaming."""
        if self.flask_thread is not None and self.flask_thread.is_alive():
            print("Stream server already running")
            return
        
        # Create Flask app
        self.flask_app = Flask(__name__)
        tracker = self  # Reference for closure
        
        @self.flask_app.route('/')
        def index():
            return """
            <html>
            <head>
                <title>Air Hockey Camera View</title>
                <style>
                    body { background: #111; text-align: center; color: #eee; font-family: sans-serif; margin: 0; padding: 20px; }
                    h2 { margin-bottom: 10px; }
                    img { border: 2px solid #555; max-width: 95%; }
                    .info { color: #888; font-size: 12px; margin-top: 10px; }
                </style>
            </head>
            <body>
                <h2>Air Hockey Defense Controller - Live View</h2>
                <img src="/video_feed">
                <p class="info">Magenta line = intercept path | Cyan box = table boundary | Red = puck | Green = paddle</p>
            </body>
            </html>
            """
        
        @self.flask_app.route('/video_feed')
        def video_feed():
            def generate():
                while tracker.show_preview and tracker.running:
                    with tracker.frame_lock:
                        if tracker.latest_frame is None:
                            time.sleep(0.01)
                            continue
                        frame = tracker.latest_frame.copy()
                    
                    # Encode as JPEG
                    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                    time.sleep(0.016)  # ~60 FPS max
            
            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
        
        def run_flask():
            # Completely suppress Flask/werkzeug output
            import logging
            import os
            
            # Disable all Flask logging
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.CRITICAL)
            log.disabled = True
            
            # Also disable Flask's own logger
            self.flask_app.logger.disabled = True
            logging.getLogger('flask').setLevel(logging.CRITICAL)
            
            # Redirect stderr temporarily during startup
            import sys
            old_stderr = sys.stderr
            sys.stderr = open(os.devnull, 'w')
            
            try:
                self.flask_app.run(host='0.0.0.0', port=self.stream_port, 
                                  threaded=True, use_reloader=False)
            except Exception as e:
                pass  # Silently handle errors
            finally:
                sys.stderr = old_stderr
        
        self.flask_thread = threading.Thread(target=run_flask, daemon=True)
        self.flask_thread.start()
        
        # Give Flask a moment to start
        time.sleep(0.1)
        print(f"Stream: http://0.0.0.0:{self.stream_port}")
    
    def stop_stream_server(self):
        """Stop Flask streaming server."""
        # Flask doesn't have a clean way to stop from another thread
        # The daemon thread will exit when main program exits
        print("Stream server will stop (view disabled)")
        self.show_preview = False
    
    def show_frame(self):
        """Legacy method - now handled by Flask streaming."""
        # This is now a no-op since we use Flask streaming
        return self.show_preview
    
    def get_puck_workspace_state(self):
        """Get puck position and velocity in workspace coordinates (mm).
        
        Returns:
            tuple: ((x_mm, y_mm), (vx_mm_s, vy_mm_s)) or (None, None) if not visible or not homed
        """
        pos, vel = self.get_puck_state()
        
        if pos is None or vel is None:
            return None, None
        
        # Convert pixel position to workspace mm
        result = self.camera_to_workspace(pos[0], pos[1])
        if result[0] is None:
            return None, None  # Not homed yet
        px_mm, py_mm = result
        
        # Convert pixel velocity to workspace velocity (mm/s)
        # Scale velocity by the same ratio as position (using HOMING values)
        if self.virtual_boundary:
            x_min, y_min, x_max, y_max = self.virtual_boundary
            frame_width = x_max - x_min
            frame_height = y_max - y_min
        else:
            frame_width = 1280
            frame_height = 720
        
        # Get workspace dimensions from homing (GROUND TRUTH)
        ws_x_range = getattr(self, 'workspace_x_max', 0) - getattr(self, 'workspace_x_min', 0)
        ws_y_range = getattr(self, 'workspace_y_max', 0) - getattr(self, 'workspace_y_min', 0)
        
        if ws_x_range <= 0 or ws_y_range <= 0:
            return None, None  # Not homed yet
        
        # Velocity scaling: pixels/frame -> mm/s (assuming ~60fps)
        # Y is inverted (camera Y down = motor Y up)
        vx_mm = (vel[0] / frame_width) * ws_x_range * 60.0
        vy_mm = -(vel[1] / frame_height) * ws_y_range * 60.0  # Invert Y
        
        return (px_mm, py_mm), (vx_mm, vy_mm)
    
    def get_paddle_workspace_pos(self):
        """Get paddle position in workspace coordinates (mm).
        
        Returns:
            tuple: (x_mm, y_mm) or None if not visible or not homed
        """
        pos = self.get_paddle_position()
        if pos is None:
            return None
        
        result = self.camera_to_workspace(pos[0], pos[1])
        # camera_to_workspace returns (None, None) if not homed
        if result[0] is None:
            return None
        return result


class MotorController:
    """RP2350 CoreXY motor controller interface."""
    
    def __init__(self, port=None):
        if port is None:
            port = find_motor_controller()
            if port is None:
                print("ERROR: Could not find RP2350 motor controller")
                sys.exit(1)
        
        self.port = port
        self.ser = None
        self.homed = False
        self.enabled = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.workspace_limits = {'x_min': 0, 'x_max': 70, 'y_min': 0, 'y_max': 35}
    
    def connect(self):
        """Connect to motor controller."""
        try:
            print(f"Connecting to motor controller at {self.port}...")
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            time.sleep(2)
            
            # Clear buffer
            while self.ser.in_waiting:
                self.ser.readline()
            
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
    
    def send_cmd(self, cmd_dict, timeout=1.0):
        """Send JSON command and get response."""
        try:
            cmd_json = json.dumps(cmd_dict) + '\n'
            self.ser.write(cmd_json.encode('utf-8'))
            self.ser.flush()
            
            # Read response
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            resp = json.loads(line)
                            
                            # Handle debug messages separately
                            if resp.get('debug') == 'motion':
                                self._print_debug_motion(resp)
                                continue  # Keep waiting for actual response
                            
                            # Update internal state
                            self.current_x = resp.get('x', self.current_x)
                            self.current_y = resp.get('y', self.current_y)
                            self.homed = resp.get('homed', self.homed)
                            self.enabled = resp.get('enabled', self.enabled)
                            return resp
                        except:
                            pass
            return None
        except Exception as e:
            print(f"Command error: {e}")
            return None
    
    def _print_debug_motion(self, msg):
        """Print formatted debug motion message."""
        x = msg.get('x', 0)
        y = msg.get('y', 0)
        vx = msg.get('vx', 0)
        vy = msg.get('vy', 0)
        speed = msg.get('speed', 0)
        dir_str = msg.get('dir', '')
        x_max = msg.get('x_max', 0)
        y_max = msg.get('y_max', 0)
        
        print(f"  [DBG] pos=({x:.1f},{y:.1f}) vel=({vx:.0f},{vy:.0f}) speed={speed:.0f} {dir_str} | limits: X[0-{x_max:.0f}] Y[0-{y_max:.0f}]")
    
    def enable(self):
        """Enable motors."""
        print("Enabling motors...")
        resp = self.send_cmd({"cmd": "enable", "state": True})
        if resp and resp.get('status') == 'ok':
            print("✓ Motors enabled")
            return True
        print("✗ Enable failed")
        return False
    
    def disable(self):
        """Disable motors."""
        resp = self.send_cmd({"cmd": "enable", "state": False})
        if resp:
            print("✓ Motors disabled")
    
    def home(self):
        """Run sensorless homing sequence."""
        print("\n" + "="*60)
        print("STARTING SENSORLESS HOMING")
        print("="*60)
        print("The gantry will:")
        print("  1. Find all workspace boundaries")
        print("  2. Return to bottom-left corner (0,0)")
        print("  3. Set workspace limits")
        print("\nThis may take 30-90 seconds depending on workspace size...")
        print("Waiting for completion...")
        print("="*60 + "\n")
        
        if not self.enabled:
            print("⚠ Warning: Motors not enabled, enabling now...")
            if not self.enable():
                return False
        
        # Send home command
        try:
            cmd_json = json.dumps({"cmd": "home"}) + '\n'
            self.ser.write(cmd_json.encode('utf-8'))
            self.ser.flush()
        except Exception as e:
            print(f"✗ Failed to send home command: {e}")
            return False
        
        # Wait for completion (no timeout - wait until done)
        print("Waiting for homing to complete (listening for 'ok' status)...")
        start_time = time.time()
        last_update = start_time
        
        while True:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            resp = json.loads(line)
                            
                            # Update position
                            self.current_x = resp.get('x', self.current_x)
                            self.current_y = resp.get('y', self.current_y)
                            
                            # Check if it's a home command response
                            if resp.get('cmd') == 'home':
                                status = resp.get('status')
                                message = resp.get('message', '')
                                
                                if status == 'in_progress':
                                    print(f"  {message}")
                                elif status == 'ok':
                                    print(f"\n✓ Homing complete! ({time.time() - start_time:.1f}s)")
                                    self.homed = True
                                    
                                    # Get workspace limits
                                    status_resp = self.send_cmd({"cmd": "status"})
                                    if status_resp:
                                        self.workspace_limits = {
                                            'x_min': status_resp.get('x_min', 0),
                                            'x_max': status_resp.get('x_max', 70),
                                            'y_min': status_resp.get('y_min', 0),
                                            'y_max': status_resp.get('y_max', 35)
                                        }
                                        print(f"  Workspace: X [{self.workspace_limits['x_min']:.1f} to {self.workspace_limits['x_max']:.1f}] mm")
                                        print(f"             Y [{self.workspace_limits['y_min']:.1f} to {self.workspace_limits['y_max']:.1f}] mm")
                                    return True
                                elif status == 'error':
                                    print(f"✗ Homing failed: {message}")
                                    return False
                        except json.JSONDecodeError:
                            pass  # Ignore non-JSON lines
                
                # Print progress every 10 seconds
                if time.time() - last_update > 10:
                    print(f"  Still homing... ({time.time() - start_time:.0f}s elapsed)")
                    last_update = time.time()
                
                time.sleep(0.05)
                
            except KeyboardInterrupt:
                print("\n⚠ Homing interrupted by user!")
                self.stop()
                return False
            except Exception as e:
                print(f"✗ Error during homing: {e}")
                return False
        
        if not self.enabled:
            print("⚠ Warning: Motors not enabled, enabling now...")
            if not self.enable():
                return False
        
        resp = self.send_cmd({"cmd": "home"}, timeout=120.0)
        
        if resp and resp.get('status') == 'ok':
            print("\n✓ Homing complete!")
            self.homed = True
            
            # Get workspace limits
            status = self.send_cmd({"cmd": "status"})
            if status:
                self.workspace_limits = {
                    'x_min': status.get('x_min', 0),
                    'x_max': status.get('x_max', 70),
                    'y_min': status.get('y_min', 0),
                    'y_max': status.get('y_max', 35)
                }
                print(f"  Workspace: X [{self.workspace_limits['x_min']:.1f} to {self.workspace_limits['x_max']:.1f}] mm")
                print(f"             Y [{self.workspace_limits['y_min']:.1f} to {self.workspace_limits['y_max']:.1f}] mm")
            return True
        else:
            print("✗ Homing failed")
            return False
    
    def stop(self):
        """Stop all motion."""
        self.send_cmd({"cmd": "stop"})
    
    def set_debug_stream(self, enabled):
        """Enable/disable debug streaming from Arduino."""
        resp = self.send_cmd({"cmd": "stream", "state": enabled})
        if resp and resp.get('status') == 'ok':
            print(f"Debug streaming: {'ON' if enabled else 'OFF'}")
            return True
        return False
    
    def set_workspace(self, x_min, x_max, y_min, y_max):
        """Set workspace limits on Arduino (syncs with homing results)."""
        resp = self.send_cmd({
            "cmd": "set_workspace",
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max
        })
        if resp and resp.get('status') == 'ok':
            self.workspace_limits = {
                'x_min': x_min, 'x_max': x_max,
                'y_min': y_min, 'y_max': y_max
            }
            self.homed = True
            print(f"Workspace synced: X[{x_min:.1f}-{x_max:.1f}] Y[{y_min:.1f}-{y_max:.1f}] mm")
            return True
        return False
    
    def calibrate_camera(self, tracker):
        """Calibrate camera-to-workspace mapping after sensorless homing.
        
        After sensorless homing establishes the physical workspace (GROUND TRUTH),
        this function moves the paddle to corners and records camera positions
        to create an accurate camera-to-workspace transform.
        
        Args:
            tracker: PuckTracker instance with camera initialized
        
        Returns:
            bool: True if successful
        """
        if not self.homed:
            print("✗ Error: Must run sensorless homing first!")
            return False
        
        print("\n" + "="*60)
        print("CAMERA CALIBRATION")
        print("="*60)
        print("Recording paddle positions at workspace corners...")
        print(f"Workspace from homing: X[0-{self.workspace_limits['x_max']:.1f}] Y[0-{self.workspace_limits['y_max']:.1f}] mm")
        print("="*60 + "\n")
        
        # Get workspace limits from homing (GROUND TRUTH)
        ws_x_max = self.workspace_limits['x_max']
        ws_y_max = self.workspace_limits['y_max']
        margin = 15.0  # Stay 15mm from edges during calibration
        
        calibration_points = []
        
        # Corner 1: Origin (0+margin, 0+margin)
        print("Moving to corner 1 (origin area)...")
        self.move_to(margin, margin)
        time.sleep(2.0)  # Wait for motion to complete
        paddle_pos = tracker.get_paddle_position()
        if paddle_pos:
            calibration_points.append(('origin', (margin, margin), paddle_pos))
            print(f"  Motor: ({margin:.1f}, {margin:.1f}) mm -> Camera: ({paddle_pos[0]:.0f}, {paddle_pos[1]:.0f}) px")
        else:
            print("  ⚠ Paddle not visible!")
        
        # Corner 2: X-max (x_max-margin, margin)
        print("Moving to corner 2 (X-max area)...")
        self.move_to(ws_x_max - margin, margin)
        time.sleep(2.0)
        paddle_pos = tracker.get_paddle_position()
        if paddle_pos:
            calibration_points.append(('x_max', (ws_x_max - margin, margin), paddle_pos))
            print(f"  Motor: ({ws_x_max - margin:.1f}, {margin:.1f}) mm -> Camera: ({paddle_pos[0]:.0f}, {paddle_pos[1]:.0f}) px")
        else:
            print("  ⚠ Paddle not visible!")
        
        # Corner 3: Y-max (margin, y_max-margin)
        print("Moving to corner 3 (Y-max area)...")
        self.move_to(margin, ws_y_max - margin)
        time.sleep(2.0)
        paddle_pos = tracker.get_paddle_position()
        if paddle_pos:
            calibration_points.append(('y_max', (margin, ws_y_max - margin), paddle_pos))
            print(f"  Motor: ({margin:.1f}, {ws_y_max - margin:.1f}) mm -> Camera: ({paddle_pos[0]:.0f}, {paddle_pos[1]:.0f}) px")
        else:
            print("  ⚠ Paddle not visible!")
        
        # Move back to center
        print("Moving to center...")
        self.move_to(ws_x_max / 2, ws_y_max / 2)
        
        # Calculate camera-to-workspace transform from calibration points
        if len(calibration_points) >= 2:
            # Use first two points to calculate scale and direction
            p1_name, p1_motor, p1_cam = calibration_points[0]
            p2_name, p2_motor, p2_cam = calibration_points[1]
            
            # Calculate pixel-to-mm ratios
            motor_dx = p2_motor[0] - p1_motor[0]
            motor_dy = p2_motor[1] - p1_motor[1]
            cam_dx = p2_cam[0] - p1_cam[0]
            cam_dy = p2_cam[1] - p1_cam[1]
            
            # Store calibration in tracker
            if cam_dx != 0:
                tracker.cam_to_ws_x_scale = motor_dx / cam_dx
            if cam_dy != 0:
                tracker.cam_to_ws_y_scale = motor_dy / cam_dy
            
            # Store reference point
            tracker.cam_ref_point = p1_cam
            tracker.ws_ref_point = p1_motor
            
            print("\n✓ Camera calibration complete!")
            print(f"  X scale: {getattr(tracker, 'cam_to_ws_x_scale', 0):.4f} mm/pixel")
            print(f"  Y scale: {getattr(tracker, 'cam_to_ws_y_scale', 0):.4f} mm/pixel")
            
            # Update tracker's workspace limits
            tracker.set_workspace_limits(0, ws_x_max, 0, ws_y_max)
            
            return True
        else:
            print("✗ Calibration failed: Not enough visible points")
            return False
    
    def move_to(self, x, y):
        """Move to absolute position (mm).
        
        Args:
            x: Target X position (mm)
            y: Target Y position (mm)
        """
        resp = self.send_cmd({"cmd": "move", "x": x, "y": y})
        return resp and resp.get('status') == 'ok'
    
    def set_velocity(self, x_vel, y_vel):
        """Set velocity (mm/s).
        
        Args:
            x_vel: X velocity (mm/s)
            y_vel: Y velocity (mm/s)
        """
        resp = self.send_cmd({"cmd": "velocity", "x_vel": x_vel, "y_vel": y_vel})
        return resp and resp.get('status') == 'ok'
    
    def localize_from_vision(self, x_mm, y_mm):
        """Set position from vision-based localization.
        
        This allows the controller to recover its position using camera tracking
        without needing to run full sensorless homing. The workspace must have
        been homed at least once to establish the boundaries.
        
        Args:
            x_mm: Paddle X position from camera (mm)
            y_mm: Paddle Y position from camera (mm)
        
        Returns:
            bool: True if successful
        """
        resp = self.send_cmd({"cmd": "localize", "x": x_mm, "y": y_mm})
        if resp and resp.get('status') == 'ok':
            print(f"✓ Position localized to ({x_mm:.1f}, {y_mm:.1f}) mm")
            self.current_x = x_mm
            self.current_y = y_mm
            return True
        else:
            print(f"✗ Localization failed: {resp.get('message', 'Unknown error') if resp else 'No response'}")
            return False
    
    def get_position(self):
        """Get current position.
        
        Returns:
            tuple: (x, y) in mm
        """
        return self.current_x, self.current_y
    
    # Directional commands for testing
    def right(self, speed=200.0):
        """Move right at speed (mm/s)."""
        print(f"RIGHT at {speed} mm/s")
        return self.set_velocity(speed, 0)
    
    def left(self, speed=200.0):
        """Move left at speed (mm/s)."""
        print(f"LEFT at {speed} mm/s")
        return self.set_velocity(-speed, 0)
    
    def up(self, speed=200.0):
        """Move up in CAMERA view (motor Y decreases)."""
        print(f"UP at {speed} mm/s")
        return self.set_velocity(0, -speed)  # Camera Y is inverted from motor Y
    
    def down(self, speed=200.0):
        """Move down in CAMERA view (motor Y increases)."""
        print(f"DOWN at {speed} mm/s")
        return self.set_velocity(0, speed)  # Camera Y is inverted from motor Y
    
    def emergency_stop(self):
        """Emergency stop - disables motors immediately."""
        try:
            if self.ser and self.ser.is_open:
                # Send stop command multiple times rapidly
                stop_cmd = b'{"cmd":"stop"}\n'
                disable_cmd = b'{"cmd":"enable","state":false}\n'
                
                for _ in range(5):
                    self.ser.write(stop_cmd)
                    self.ser.flush()
                
                time.sleep(0.05)
                
                for _ in range(5):
                    self.ser.write(disable_cmd)
                    self.ser.flush()
                
                time.sleep(0.1)
        except Exception as e:
            print(f"Emergency stop error: {e}")
    
    def close(self):
        """Close connection."""
        if self.ser and self.ser.is_open:
            print("\nStopping motors...")
            self.emergency_stop()
            time.sleep(0.2)
            self.ser.close()
            print("Motor controller disconnected")


class DefenseController:
    """Main defense controller integrating vision and motors.
    
    This controller:
    1. Gathers context from camera (puck position, velocity, paddle position)
    2. Calculates optimal defensive position
    3. Sends velocity commands to Arduino
    4. Lets Arduino handle safety (stall guard, boundary clamping, kinematics)
    """
    
    def __init__(self, motor_controller, puck_tracker):
        self.motor = motor_controller
        self.tracker = puck_tracker
        self.auto_mode = False
        self.defense_thread = None
        self.running = False
        
        # Defense strategy parameters
        self.max_speed = 350.0      # Max velocity command (mm/s)
        self.edge_margin = 10.0     # Stay this many mm away from workspace edges (matches Arduino)
        
        # Control gains for position tracking (PD controller)
        self.kp = 20.0  # Proportional gain - higher = faster response
        self.kd = 3.0   # Derivative gain - damping to reduce oscillation
        
        # Suppress frequent status prints (keeps terminal clean for commands)
        self.verbose = False
        
        # State tracking
        self.last_puck_pos = None
        self.last_puck_vel = None
        self.last_paddle_pos = None
    
    def start_auto_defense(self):
        """Start autonomous defense mode using camera pixel tracking."""
        if not self.tracker.virtual_boundary:
            print("⚠ Error: No camera boundary set!")
            print("  Use 'setcorner' at each corner of paddle's travel first.")
            return False
        
        bx_min, by_min, bx_max, by_max = self.tracker.virtual_boundary
        print(f"Auto defense ON - CAMERA PIXEL TRACKING mode")
        print(f"  Camera boundary: ({bx_min}, {by_min}) to ({bx_max}, {by_max})")
        print(f"  Size: {bx_max - bx_min} x {by_max - by_min} pixels")
        print(f"  Max speed: {self.max_speed:.0f}mm/s")
        print(f"  Using camera as feedback (no mm conversion)")
        
        self.auto_mode = True
        self.running = True
        self.defense_thread = threading.Thread(target=self._defense_loop, daemon=True)
        self.defense_thread.start()
        return True
    
    def stop_auto_defense(self):
        """Stop autonomous defense mode."""
        was_running = self.auto_mode
        self.auto_mode = False
        self.running = False
        if self.defense_thread:
            self.defense_thread.join(timeout=1)
        self.motor.stop()
        if was_running:
            print("Auto defense OFF")
    
    def _calculate_intercept_position(self, puck_pos, puck_vel, paddle_pos):
        """Calculate optimal intercept position for the paddle.
        
        Strategy: Move to intercept the puck along its trajectory.
        The paddle can move freely on both axes within its workspace.
        Uses homing-derived workspace limits as ground truth.
        
        Args:
            puck_pos: (x, y) in workspace mm
            puck_vel: (vx, vy) in mm/s
            paddle_pos: (x, y) current paddle position in workspace mm
            
        Returns:
            (target_x, target_y): Position to move to, or None if no action needed
        """
        px, py = puck_pos
        vx, vy = puck_vel
        
        # Use homing-derived workspace limits (ground truth)
        ws_x_min = self.motor.workspace_limits['x_min']
        ws_x_max = self.motor.workspace_limits['x_max']
        ws_y_min = self.motor.workspace_limits['y_min']
        ws_y_max = self.motor.workspace_limits['y_max']
        
        # Safety margins from edges
        x_min = ws_x_min + self.edge_margin
        x_max = ws_x_max - self.edge_margin
        y_min = ws_y_min + self.edge_margin
        y_max = ws_y_max - self.edge_margin
        
        puck_speed = np.sqrt(vx**2 + vy**2)
        
        # If puck is slow or stationary, move directly toward it
        if puck_speed < 20.0:
            target_x = max(x_min, min(x_max, px))
            target_y = max(y_min, min(y_max, py))
            return (target_x, target_y)
        
        # If puck is moving toward our goal (negative X = toward left/goal)
        if vx < -10.0:
            # Calculate time for puck to reach our defensive zone (left third)
            intercept_x = max(x_min, ws_x_max * 0.3)  # Intercept in left 30%
            
            if px > intercept_x:
                time_to_intercept = (intercept_x - px) / vx
                if 0 < time_to_intercept < 1.5:
                    # Predict where puck will be at intercept time
                    target_y = py + vy * time_to_intercept
                    target_y = max(y_min, min(y_max, target_y))
                    return (intercept_x, target_y)
        
        # Puck moving away or across - chase it directly
        # Project puck position forward a bit (0.2 seconds)
        prediction_time = 0.2
        future_x = px + vx * prediction_time
        future_y = py + vy * prediction_time
        
        # Clamp to workspace
        target_x = max(x_min, min(x_max, future_x))
        target_y = max(y_min, min(y_max, future_y))
        
        return (target_x, target_y)
    
    def _defense_loop(self):
        """Main defense control loop using CAMERA PIXELS directly.
        
        Strategy:
        1. Get puck and paddle positions in CAMERA PIXELS
        2. Calculate error in pixels
        3. Send velocity proportional to pixel error
        4. Camera provides closed-loop feedback
        
        No mm conversion needed - camera is the ground truth!
        """
        puck_was_in_play = True
        last_error_x = 0.0
        last_error_y = 0.0
        
        # Get camera boundary (from setcorner)
        if not self.tracker.virtual_boundary:
            print("⚠ Error: No camera boundary set! Use 'setcorner' first.")
            self.auto_mode = False
            return
        
        bx_min, by_min, bx_max, by_max = self.tracker.virtual_boundary
        center_px = (bx_min + bx_max) / 2.0
        center_py = (by_min + by_max) / 2.0
        
        # Pixel-to-velocity gain (tune this for responsiveness)
        # Higher = faster response, lower = smoother
        PIXEL_GAIN = 1.0  # mm/s per pixel of error
        
        while self.running and self.auto_mode:
            try:
                # === GET POSITIONS IN CAMERA PIXELS ===
                puck_pos = self.tracker.get_puck_state()[0]  # (x, y) in pixels
                paddle_pos = self.tracker.get_paddle_position()  # (x, y) in pixels
                
                if paddle_pos is None:
                    # Can't see paddle - stop
                    self.motor.stop()
                    time.sleep(0.05)
                    continue
                
                paddle_x, paddle_y = paddle_pos
                
                # === DETERMINE TARGET IN PIXELS ===
                if puck_pos is None:
                    # No puck - return to center of boundary
                    target_x = center_px
                    target_y = center_py
                else:
                    # Chase the puck!
                    puck_x, puck_y = puck_pos
                    
                    # Clamp puck position to boundary
                    target_x = max(bx_min + 20, min(bx_max - 20, puck_x))
                    target_y = max(by_min + 20, min(by_max - 20, puck_y))
                
                # === CALCULATE ERROR IN PIXELS ===
                error_x = target_x - paddle_x
                error_y = target_y - paddle_y
                
                # PD control
                d_error_x = error_x - last_error_x
                d_error_y = error_y - last_error_y
                last_error_x = error_x
                last_error_y = error_y
                
                # Calculate velocity command
                # IMPORTANT: Camera Y is inverted from motor Y
                # Camera: Y increases downward
                # Motor: Y increases upward (typically)
                # So we NEGATE vy
                vx = (error_x * PIXEL_GAIN * self.kp) + (d_error_x * self.kd)
                vy = -((error_y * PIXEL_GAIN * self.kp) + (d_error_y * self.kd))  # INVERTED
                
                # Clamp velocity
                speed = np.sqrt(vx**2 + vy**2)
                if speed > self.max_speed:
                    scale = self.max_speed / speed
                    vx *= scale
                    vy *= scale
                
                # Send velocity command
                self.motor.set_velocity(vx, vy)
                
            except Exception as e:
                print(f"Defense error: {e}")
                import traceback
                traceback.print_exc()
            
            time.sleep(0.025)  # 40 Hz control loop
    
    def _defense_loop_old(self):
        """Old defense loop using mm conversion (kept for reference)."""
        puck_was_in_play = True
        last_error_x = 0.0
        last_error_y = 0.0
        
        ws_x_min = self.motor.workspace_limits['x_min']
        ws_x_max = self.motor.workspace_limits['x_max']
        ws_y_min = self.motor.workspace_limits['y_min']
        ws_y_max = self.motor.workspace_limits['y_max']
        
        while self.running and self.auto_mode:
            try:
                puck_in_play = self.tracker.is_puck_in_play()
                
                if not puck_in_play:
                    if puck_was_in_play:
                        if self.verbose:
                            print("Puck out of play - returning to center...")
                        puck_was_in_play = False
                    
                    center_x = (ws_x_max + ws_x_min) / 2.0
                    center_y = (ws_y_max + ws_y_min) / 2.0
                    
                    paddle_pos = self.tracker.get_paddle_workspace_pos()
                    if paddle_pos:
                        current_x, current_y = paddle_pos
                    else:
                        current_x, current_y = center_x, center_y
                    
                    error_x = center_x - current_x
                    error_y = center_y - current_y
                    
                    vx = error_x * 5.0
                    vy = error_y * 5.0
                    
                    speed = np.sqrt(vx**2 + vy**2)
                    if speed > 100:
                        scale = 100 / speed
                        vx *= scale
                        vy *= scale
                    
                    self.motor.set_velocity(vx, vy)
                    time.sleep(0.05)
                    continue
                else:
                    if not puck_was_in_play:
                        if self.verbose:
                            print("Puck in play - defending!")
                        puck_was_in_play = True
                
                # Get puck state in workspace coordinates (uses homing limits)
                puck_pos, puck_vel = self.tracker.get_puck_workspace_state()
                
                if puck_pos is None:
                    time.sleep(0.02)
                    continue
                
                self.last_puck_pos = puck_pos
                self.last_puck_vel = puck_vel
                
                # Get paddle position from camera (in homing workspace coords)
                paddle_pos = self.tracker.get_paddle_workspace_pos()
                if paddle_pos:
                    self.last_paddle_pos = paddle_pos
                    current_x, current_y = paddle_pos
                else:
                    # No paddle detected - skip this frame
                    time.sleep(0.02)
                    continue
                
                # === CALCULATE OPTIMAL INTERCEPT POSITION ===
                target = self._calculate_intercept_position(puck_pos, puck_vel, paddle_pos)
                
                if target is not None:
                    target_x, target_y = target
                    
                    # PD control for both axes
                    error_x = target_x - current_x
                    error_y = target_y - current_y
                    
                    d_error_x = error_x - last_error_x
                    d_error_y = error_y - last_error_y
                    last_error_x = error_x
                    last_error_y = error_y
                    
                    vx = (self.kp * error_x) + (self.kd * d_error_x)
                    vy = (self.kp * error_y) + (self.kd * d_error_y)
                    
                    # Clamp velocity magnitude
                    speed = np.sqrt(vx**2 + vy**2)
                    if speed > self.max_speed:
                        scale = self.max_speed / speed
                        vx *= scale
                        vy *= scale
                    
                    # Edge safety using homing limits
                    edge_dist = 10.0  # Safety distance from edge (mm)
                    
                    # Don't accelerate toward edges we're close to
                    if current_x < (ws_x_min + edge_dist) and vx < 0:
                        vx = 0
                    if current_x > (ws_x_max - edge_dist) and vx > 0:
                        vx = 0
                    if current_y < (ws_y_min + edge_dist) and vy < 0:
                        vy = 0
                    if current_y > (ws_y_max - edge_dist) and vy > 0:
                        vy = 0
                    
                    # === SEND VELOCITY COMMAND TO ARDUINO ===
                    self.motor.set_velocity(vx, vy)
                
            except Exception as e:
                print(f"Defense error: {e}")
                import traceback
                traceback.print_exc()
            
            time.sleep(0.025)  # 40 Hz control loop


def main():
    """Main program."""
    print("="*60)
    print("AIR HOCKEY DEFENSE CONTROLLER")
    print("="*60)
    print("\nInitializing...\n")
    
    # Initialize motor controller
    motor = MotorController()
    tracker = None
    defense = None
    
    # Setup signal handlers for clean shutdown
    import signal
    
    def signal_handler(sig, frame):
        """Handle Ctrl+C and other termination signals."""
        print("\n\n⚠ Interrupt signal received! Stopping motors...")
        try:
            if defense:
                defense.stop_auto_defense()
            if motor:
                motor.emergency_stop()
        except Exception as e:
            print(f"Error during emergency stop: {e}")
        finally:
            sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Kill signal
    
    if not motor.connect():
        print("Failed to connect to motor controller. Exiting.")
        return
    
    # Initialize puck tracker (optional - camera may not be available)
    tracker = PuckTracker()
    camera_available = tracker.start()
    
    if not camera_available:
        print("\n⚠ WARNING: Camera not available")
        print("   - Autonomous defense mode will NOT work")
        print("   - Manual control will work normally")
        print("   - Possible causes:")
        print("     1. Another process using camera (kill stream_camera.py)")
        print("     2. Camera not connected")
        print("     3. Permission issues")
        print("\n   Continuing in MANUAL MODE ONLY...\n")
    
    # Initialize defense controller
    defense = DefenseController(motor, tracker)
    
    try:
        # Enable motors
        if not motor.enable():
            print("Failed to enable motors. Exiting.")
            return
        
        # Prompt for homing
        print("\n" + "="*60)
        print("SETUP")
        print("="*60)
        if not motor.homed:
            choice = input("Run sensorless homing calibration? [Y/n]: ").strip().lower()
            if choice != 'n':
                if not motor.home():
                    print("Homing failed. Exiting for safety.")
                    return
                
                # Update tracker with actual workspace limits from motor controller
                if camera_available and tracker:
                    tracker.set_workspace_limits(
                        motor.workspace_limits['x_min'],
                        motor.workspace_limits['x_max'],
                        motor.workspace_limits['y_min'],
                        motor.workspace_limits['y_max']
                    )
        
        print("\n" + "="*60)
        print("READY!")
        print("="*60)
        print("\nCommands:")
        print("  auto         - Start autonomous defense")
        print("  stop         - Stop all motion")
        print("  debug        - Toggle Arduino motion debug output")
        print("  view         - Toggle web stream (http://<ip>:5001)")
        print("  right/left/up/down - Manual movement (200mm/s)")
        print("  center       - MOVE to center of workspace")
        print("  reset        - SET position to center (no motion)")
        print("  home         - Run sensorless homing")
        print("  setcorner    - Record paddle position as boundary corner")
        print("  clearcorners - Clear recorded corners")
        print("  syncws       - Sync motor workspace to physical travel")
        print("  status       - Show current position and limits")
        print("  quit         - Exit")
        print("\nSetup: home -> view -> setcorner (x4) -> syncws -> auto")
        print("Press Ctrl+C at any time to emergency stop")
        print("="*60 + "\n")
        
        # Main command loop
        while True:
            try:
                cmd = input(">>> ").strip().lower()
                
                if not cmd:
                    continue
                
                if cmd in ['quit', 'exit', 'q']:
                    break
                
                elif cmd == 'auto':
                    if not camera_available:
                        print("⚠ Error: Camera not available. Cannot start autonomous mode.")
                        print("   Run manual commands or fix camera connection.")
                    else:
                        defense.start_auto_defense()
                
                elif cmd == 'stop':
                    defense.stop_auto_defense()
                    motor.stop()
                
                elif cmd == 'view':
                    if not camera_available:
                        print("⚠ Error: Camera not available.")
                    else:
                        tracker.toggle_preview()
                
                elif cmd == 'localize':
                    if not camera_available:
                        print("⚠ Error: Camera not available. Cannot localize.")
                    else:
                        print("Looking for green paddle in camera view...")
                        # Wait a moment for tracking to stabilize
                        time.sleep(0.5)
                        
                        paddle_pos = tracker.get_paddle_position()
                        if paddle_pos is None:
                            print("✗ Cannot see green paddle. Make sure it's visible in camera view.")
                        else:
                            # Convert camera coordinates to workspace coordinates
                            cam_x, cam_y = paddle_pos
                            workspace_x, workspace_y = tracker.camera_to_workspace(cam_x, cam_y)
                            print(f"  Camera position: ({cam_x:.0f}, {cam_y:.0f}) pixels")
                            print(f"  Workspace position: ({workspace_x:.1f}, {workspace_y:.1f}) mm")
                            
                            # Update motor controller position
                            if motor.localize_from_vision(workspace_x, workspace_y):
                                print("✓ Position recovered! You can now use velocity commands.")
                            else:
                                print("✗ Failed to update position. Make sure workspace was homed at least once.")
                
                elif cmd == 'right':
                    motor.right()
                
                elif cmd == 'left':
                    motor.left()
                
                elif cmd == 'up':
                    motor.up()
                
                elif cmd == 'down':
                    motor.down()
                
                elif cmd == 'center':
                    center_x = (motor.workspace_limits['x_max'] + motor.workspace_limits['x_min']) / 2.0
                    center_y = (motor.workspace_limits['y_max'] + motor.workspace_limits['y_min']) / 2.0
                    print(f"Moving to center ({center_x:.1f}, {center_y:.1f})...")
                    motor.move_to(center_x, center_y)
                
                elif cmd == 'reset':
                    # Reset position to center WITHOUT moving (for when position tracking is wrong)
                    center_x = (motor.workspace_limits['x_max'] + motor.workspace_limits['x_min']) / 2.0
                    center_y = (motor.workspace_limits['y_max'] + motor.workspace_limits['y_min']) / 2.0
                    motor.current_x = center_x
                    motor.current_y = center_y
                    # Also tell Arduino
                    motor.send_cmd({"cmd": "localize", "x": center_x, "y": center_y})
                    print(f"Position RESET to center ({center_x:.1f}, {center_y:.1f}) - no motion")
                
                elif cmd == 'setcorner':
                    # Record paddle position as a corner of the virtual boundary
                    if not camera_available:
                        print("⚠ Error: Camera not available")
                    else:
                        paddle_pos = tracker.get_paddle_position()
                        if paddle_pos is None:
                            print("⚠ Error: Paddle not visible!")
                        else:
                            if not hasattr(tracker, '_corners'):
                                tracker._corners = []
                            # Store as integers
                            corner = (int(paddle_pos[0]), int(paddle_pos[1]))
                            tracker._corners.append(corner)
                            print(f"Corner {len(tracker._corners)} recorded: pixel ({corner[0]}, {corner[1]})")
                            if len(tracker._corners) >= 2:
                                # Calculate bounding box from all corners
                                xs = [c[0] for c in tracker._corners]
                                ys = [c[1] for c in tracker._corners]
                                new_boundary = (int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys)))
                                tracker.virtual_boundary = new_boundary
                                tracker.MANUAL_BOUNDARY = new_boundary
                                print(f"✓ Virtual boundary updated: ({new_boundary[0]}, {new_boundary[1]}) to ({new_boundary[2]}, {new_boundary[3]})")
                                print(f"  Size: {new_boundary[2]-new_boundary[0]} x {new_boundary[3]-new_boundary[1]} pixels")
                
                elif cmd == 'clearcorners':
                    if hasattr(tracker, '_corners'):
                        tracker._corners = []
                    print("Corners cleared - use 'setcorner' to record new ones")
                
                elif cmd == 'syncws':
                    # After setting corners, sync motor workspace to match
                    # This tells the motor its ACTUAL physical travel limits
                    print("\nSYNC WORKSPACE")
                    print("This sets motor workspace to match paddle's actual travel.")
                    print("Enter the physical dimensions (in mm) of your paddle's travel:")
                    try:
                        x_travel = float(input("  X travel (mm, e.g. 50): ").strip() or "50")
                        y_travel = float(input("  Y travel (mm, e.g. 30): ").strip() or "30")
                        
                        # Update motor workspace
                        motor.set_workspace(0, x_travel, 0, y_travel)
                        
                        # Update tracker workspace
                        tracker.set_workspace_limits(0, x_travel, 0, y_travel)
                        
                        print(f"\n✓ Workspace synced: X[0-{x_travel}mm] Y[0-{y_travel}mm]")
                        print("  Camera boundary will now map to this workspace.")
                        print("  Auto defense should now work correctly!")
                    except ValueError:
                        print("Invalid input - use numbers only")
                
                elif cmd == 'home':
                    if motor.home():
                        # Update tracker with new workspace limits
                        if camera_available and tracker:
                            tracker.set_workspace_limits(
                                motor.workspace_limits['x_min'],
                                motor.workspace_limits['x_max'],
                                motor.workspace_limits['y_min'],
                                motor.workspace_limits['y_max']
                            )
                
                elif cmd == 'camera_home':
                    if not camera_available:
                        print("⚠ Error: Camera not available. Cannot perform camera-guided homing.")
                    else:
                        motor.camera_guided_home(tracker)
                
                elif cmd == 'debug':
                    # Toggle debug streaming
                    if not hasattr(motor, '_debug_enabled'):
                        motor._debug_enabled = False
                    motor._debug_enabled = not motor._debug_enabled
                    motor.set_debug_stream(motor._debug_enabled)
                
                elif cmd == 'verbose':
                    defense.verbose = not defense.verbose
                    print(f"Verbose mode: {'ON' if defense.verbose else 'OFF'}")
                
                elif cmd == 'calibrate':
                    # Run camera calibration after homing
                    if not camera_available:
                        print("⚠ Error: Camera not available")
                    elif not motor.homed:
                        print("⚠ Error: Run 'home' first to establish workspace limits")
                    else:
                        motor.calibrate_camera(tracker)
                
                elif cmd == 'status':
                    print(f"Position: ({motor.current_x:.1f}, {motor.current_y:.1f}) mm")
                    print(f"Workspace: X[{motor.workspace_limits['x_min']:.1f}-{motor.workspace_limits['x_max']:.1f}] Y[{motor.workspace_limits['y_min']:.1f}-{motor.workspace_limits['y_max']:.1f}] mm")
                    print(f"Homed: {motor.homed}, Enabled: {motor.enabled}")
                
                else:
                    print(f"Unknown command: {cmd}")
                    
            except KeyboardInterrupt:
                print("\n\n⚠ Ctrl+C - Emergency stop!")
                defense.stop_auto_defense()
                motor.emergency_stop()
                break
            
            except EOFError:
                print("\nEOF - Exiting")
                break
    
    finally:
        print("\nShutting down...")
        if defense:
            defense.stop_auto_defense()
        if tracker:
            tracker.stop()
        if motor:
            motor.emergency_stop()
            time.sleep(0.2)  # Give time for stop commands to send
            motor.close()
        print("Done.")


if __name__ == "__main__":
    main()
