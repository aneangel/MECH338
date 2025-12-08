#!/usr/bin/env python3
"""
Stream ZED camera to web browser with Kalman Filter Puck Tracking
Access at: http://<jetson-ip>:5000
"""
import pyzed.sl as sl
import cv2
import numpy as np
from flask import Flask, Response
import threading
from collections import deque
import time

app = Flask(__name__)

# Global variables
camera_running = False
current_frame = None
frame_count = 0 
frame_lock = threading.Lock()

# Crop region (User defined)
CROP_REGION = (236, 115, 551, 303)

# Display settings
DISPLAY_SCALE = 3 

# HSV color ranges
GREEN_LOWER = np.array([35, 100, 100])
GREEN_UPPER = np.array([85, 255, 255])

# Red ranges
RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 100])
RED_UPPER2 = np.array([180, 255, 255])

MIN_PUCK_AREA = 80 # Slightly lowered to catch fast moving small pucks

class KalmanTracker:
    def __init__(self, id_color="Object"):
        self.id = id_color
        # Initialize Kalman Filter
        # 4 dynamic params: x, y, vx, vy (position and velocity)
        # 2 measurement params: x, y (we only measure position)
        self.kf = cv2.KalmanFilter(4, 2)
        
        # Measurement Matrix (H): We measure x and y directly
        self.kf.measurementMatrix = np.array([[1,0,0,0],
                                              [0,1,0,0]], np.float32)
        
        # Transition Matrix (F): Defines how state changes. 
        # x = x + vx, y = y + vy
        self.kf.transitionMatrix = np.array([[1,0,1,0],
                                             [0,1,0,1],
                                             [0,0,1,0],
                                             [0,0,0,1]], np.float32)
        
        # Process Noise (Q): How jerky is the object really? 
        # Lower = Smoother lines, Higher = Reacts faster to turns
        self.kf.processNoiseCov = np.array([[1,0,0,0],
                                            [0,1,0,0],
                                            [0,0,5,0],
                                            [0,0,0,5]], np.float32) * 0.03

        # Measurement Noise (R): How noisy is the camera?
        self.kf.measurementNoiseCov = np.identity(2, np.float32) * 0.1
        
        self.found = False
        self.frames_missing = 0
        self.max_missing = 10
        self.center = None
        self.bbox = None
        self.velocity = (0, 0)

    def update(self, measurement_center, measurement_bbox):
        """
        Update the filter. 
        If measurement is None, we predict based on last velocity.
        """
        # 1. PREDICT phase (Project state ahead)
        prediction = self.kf.predict()
        pred_x, pred_y = int(prediction[0, 0]), int(prediction[1, 0])
        
        # 2. CORRECT phase (Update with actual data if available)
        if measurement_center is not None:
            self.found = True
            self.frames_missing = 0
            
            # Feed measurement to Kalman
            meas = np.array([[np.float32(measurement_center[0])], 
                             [np.float32(measurement_center[1])]])
            self.kf.correct(meas)
            
            # Update internal state with corrected data
            # We use the Filter's position (which is a mix of prediction + measurement)
            # This effectively removes jitter
            state = self.kf.statePost
            self.center = (int(state[0, 0]), int(state[1, 0]))
            self.velocity = (float(state[2, 0]), float(state[3, 0]))
            self.bbox = measurement_bbox
            
        else:
            # No measurement: Trust the prediction (coast)
            self.frames_missing += 1
            if self.frames_missing < self.max_missing:
                self.center = (pred_x, pred_y)
                # Keep old bbox but move it
                if self.bbox:
                    w, h = self.bbox[2], self.bbox[3]
                    self.bbox = (pred_x - w//2, pred_y - h//2, w, h)
            else:
                self.found = False

    def get_prediction_points(self, steps=15):
        """Return future points based on stable Kalman velocity"""
        if not self.found or self.center is None:
            return []
            
        points = []
        cx, cy = self.center
        vx, vy = self.velocity
        
        # Only draw if moving fast enough to avoid "jittery static lines"
        speed = np.sqrt(vx**2 + vy**2)
        if speed < 1.0: 
            return []

        for i in range(1, steps + 1):
            # Predict i frames into the future
            px = int(cx + vx * i * 2) # Multiply by 2 to visualize further ahead
            py = int(cy + vy * i * 2)
            points.append((px, py))
            
        return points

# Create Kalman trackers
green_tracker = KalmanTracker("Green")
red_tracker = KalmanTracker("Red")


def detect_puck(frame_hsv, lower, upper, lower2=None, upper2=None):
    """Detect a puck with circularity check."""
    mask = cv2.inRange(frame_hsv, lower, upper)
    if lower2 is not None and upper2 is not None:
        mask2 = cv2.inRange(frame_hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask, mask2)
    
    # Clean up mask
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_cnt = None
    max_area = 0
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_PUCK_AREA:
            continue
            
        # --- SHAPE CHECK ---
        # Calculate circularity: 4 * pi * Area / Perimeter^2
        # A perfect circle is 1.0. A messy blob (hand) is much lower.
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0: continue
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        
        # Pucks are usually > 0.7 circularity. Hands/Shadows are usually < 0.5
        if circularity > 0.6:
            if area > max_area:
                max_area = area
                best_cnt = cnt

    if best_cnt is None:
        return None, None
    
    x, y, w, h = cv2.boundingRect(best_cnt)
    center = (x + w // 2, y + h // 2)
    bbox = (x, y, w, h)
    
    return center, bbox


def draw_tracking(frame, tracker, color, label):
    """Draw using Kalman state."""
    if not tracker.found:
        return
    
    center = tracker.center
    bbox = tracker.bbox
    
    if center is None or bbox is None: return
    
    x, y, w, h = bbox
    scale = DISPLAY_SCALE
    
    sx, sy, sw, sh = x * scale, y * scale, w * scale, h * scale
    sc = (center[0] * scale, center[1] * scale)
    
    # Draw BBox
    cv2.rectangle(frame, (sx, sy), (sx + sw, sy + sh), color, 2)
    cv2.putText(frame, label, (sx, sy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    # Draw Trajectory Prediction (Smooth line)
    future_points = tracker.get_prediction_points()
    if len(future_points) > 1:
        # Scale points
        scaled_pts = [(int(p[0]*scale), int(p[1]*scale)) for p in future_points]
        
        # Draw as a polyline
        cv2.polylines(frame, [np.array(scaled_pts)], False, color, 2, cv2.LINE_AA)
        
        # Draw end dot
        cv2.circle(frame, scaled_pts[-1], 4, color, -1)


def camera_thread():
    global camera_running, current_frame, frame_count
    
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.VGA
    init_params.camera_fps = 100
    init_params.depth_mode = sl.DEPTH_MODE.NONE
    
    err = zed.open(init_params)
    
    # Allow camera to run even with calibration warnings (common for 2D-only usage)
    if err != sl.ERROR_CODE.SUCCESS and err != sl.ERROR_CODE.POTENTIAL_CALIBRATION_ISSUE:
        print(f"Fatal error opening camera: {err}")
        return
    
    if err == sl.ERROR_CODE.POTENTIAL_CALIBRATION_ISSUE:
        print("Warning: Calibration issue detected, but continuing (depth disabled anyway)...")
    
    print("Camera opened successfully")
    
    # Camera Settings - Fixed exposure/gain prevents flickering
    zed.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 8)
    zed.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, 5)
    zed.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 5)
    
    image = sl.Mat()
    runtime_params = sl.RuntimeParameters()
    
    camera_running = True
    
    while camera_running:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            
            # IMPORTANT: Copy immediately - get_data() returns reference to internal buffer
            frame_bgra = image.get_data().copy()
            
            # Convert BGRA to BGR properly
            frame_bgr_full = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2BGR)
            
            # Crop to table region
            x1, y1, x2, y2 = CROP_REGION
            frame_bgr = frame_bgr_full[y1:y2, x1:x2]
            
            # Convert to HSV for color detection
            frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            
            # 1. DETECT
            green_c, green_b = detect_puck(frame_hsv, GREEN_LOWER, GREEN_UPPER)
            red_c, red_b = detect_puck(frame_hsv, RED_LOWER1, RED_UPPER1, RED_LOWER2, RED_UPPER2)
            
            # 2. UPDATE KALMAN (Predicts if detection misses, Corrects if found)
            green_tracker.update(green_c, green_b)
            red_tracker.update(red_c, red_b)
            
            # 3. DRAW
            h, w = frame_bgr.shape[:2]
            frame_scaled = cv2.resize(frame_bgr, (w * DISPLAY_SCALE, h * DISPLAY_SCALE), 
                                      interpolation=cv2.INTER_NEAREST)
            
            draw_tracking(frame_scaled, green_tracker, (0, 255, 0), "Hockey Puck")
            draw_tracking(frame_scaled, red_tracker, (0, 0, 255), "Hockey Puck")
            
            # 4. UPDATE GLOBAL
            with frame_lock:
                current_frame = frame_scaled.copy()
                frame_count += 1
    
    zed.close()
    print("Camera closed")


def generate_frames():
    """Generate frames for streaming"""
    while True:
        # Get frame with lock
        with frame_lock:
            if current_frame is None:
                pass  # Will sleep outside lock
            else:
                frame = current_frame.copy()
                # Encode and yield
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        
        # Sleep OUTSIDE lock to let camera thread update
        time.sleep(0.01)

@app.route('/')
def index():
    return """
    <html>
    <head>
        <title>ZED Tracking</title>
        <style>body{background:#111; text-align:center; color:#eee; font-family:sans-serif;}</style>
    </head>
    <body>
        <h2>ZED 2i Kalman Tracking</h2>
        <img src="/video_feed" style="border:2px solid #555; width:75%;">
    </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/tracking_data')
def tracking_data():
    """Return JSON tracking data for motor controller integration."""
    data = {
        'green': {
            'visible': green_tracker.found,
            'x': green_tracker.center[0] if green_tracker.center else 0,
            'y': green_tracker.center[1] if green_tracker.center else 0,
            'vx': green_tracker.velocity[0] if green_tracker.velocity else 0,
            'vy': green_tracker.velocity[1] if green_tracker.velocity else 0
        },
        'red': {
            'visible': red_tracker.found,
            'x': red_tracker.center[0] if red_tracker.center else 0,
            'y': red_tracker.center[1] if red_tracker.center else 0,
            'vx': red_tracker.velocity[0] if red_tracker.velocity else 0,
            'vy': red_tracker.velocity[1] if red_tracker.velocity else 0
        }
    }
    return data

if __name__ == '__main__':
    cam_thread = threading.Thread(target=camera_thread, daemon=True)
    cam_thread.start()
    time.sleep(2)
    app.run(host='0.0.0.0', port=5000, threaded=True)