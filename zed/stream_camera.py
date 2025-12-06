#!/usr/bin/env python3
"""
Stream ZED camera to web browser with puck tracking
Access at: http://<jetson-ip>:5000
"""
import pyzed.sl as sl
import cv2
import numpy as np
from flask import Flask, Response
import threading
from collections import deque

app = Flask(__name__)

# Global variables
camera_running = False
current_frame = None
frame_lock = threading.Lock()

# Crop region for the table/workspace (adjust these values as needed)
# Format: (x_start, y_start, x_end, y_end) in pixels for VGA (672x376)
CROP_REGION = (236, 115, 551, 303)  # Scaled for VGA resolution

# Display scale factor (scales up the cropped image for viewing)
DISPLAY_SCALE = 3  # 3x larger for better visibility

# HSV color ranges for puck detection (adjust if needed based on lighting)
# Green puck
GREEN_LOWER = np.array([35, 100, 100])
GREEN_UPPER = np.array([85, 255, 255])

# Red puck (red wraps around in HSV, so we need two ranges)
RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 100])
RED_UPPER2 = np.array([180, 255, 255])

# Tracking history (stores last N positions for trajectory prediction)
HISTORY_LENGTH = 10
green_history = deque(maxlen=HISTORY_LENGTH)
red_history = deque(maxlen=HISTORY_LENGTH)

# Minimum contour area to be considered a puck (filters noise)
MIN_PUCK_AREA = 100


def detect_puck(frame_hsv, lower, upper, lower2=None, upper2=None):
    """Detect a puck by color and return its center and bounding box."""
    # Create mask for the color
    mask = cv2.inRange(frame_hsv, lower, upper)
    
    # For red, combine both ranges
    if lower2 is not None and upper2 is not None:
        mask2 = cv2.inRange(frame_hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask, mask2)
    
    # Clean up mask with morphological operations
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, None
    
    # Find the largest contour (should be the puck)
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    
    if area < MIN_PUCK_AREA:
        return None, None
    
    # Get bounding box and center
    x, y, w, h = cv2.boundingRect(largest)
    center = (x + w // 2, y + h // 2)
    bbox = (x, y, w, h)
    
    return center, bbox


def predict_trajectory(history, num_points=5):
    """Predict future positions based on position history."""
    if len(history) < 2:
        return []
    
    # Get recent positions
    positions = list(history)
    
    # Calculate average velocity from recent positions
    if len(positions) >= 2:
        # Use last few positions to calculate velocity
        recent = positions[-min(5, len(positions)):]
        
        # Calculate velocity components
        vx = (recent[-1][0] - recent[0][0]) / len(recent)
        vy = (recent[-1][1] - recent[0][1]) / len(recent)
        
        # Predict future positions
        predictions = []
        last_pos = positions[-1]
        for i in range(1, num_points + 1):
            pred_x = int(last_pos[0] + vx * i * 3)  # Scale factor for visibility
            pred_y = int(last_pos[1] + vy * i * 3)
            predictions.append((pred_x, pred_y))
        
        return predictions
    
    return []


def draw_tracking(frame, center, bbox, history, predictions, color, label):
    """Draw bounding box, trail, and predicted trajectory."""
    if center is None:
        return
    
    x, y, w, h = bbox
    scale = DISPLAY_SCALE
    
    # Scale coordinates for display
    sx, sy, sw, sh = x * scale, y * scale, w * scale, h * scale
    sc = (center[0] * scale, center[1] * scale)
    
    # Draw bounding box
    cv2.rectangle(frame, (sx, sy), (sx + sw, sy + sh), color, 2)
    
    # Draw label
    cv2.putText(frame, label, (sx, sy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    # Draw center point
    cv2.circle(frame, sc, 5, color, -1)
    
    # Draw trail (past positions)
    pts = [(p[0] * scale, p[1] * scale) for p in history]
    for i in range(1, len(pts)):
        alpha = i / len(pts)  # Fade effect
        thickness = int(2 * alpha) + 1
        cv2.line(frame, pts[i-1], pts[i], color, thickness)
    
    # Draw predicted trajectory (dashed line effect)
    if predictions:
        prev = sc
        for i, pred in enumerate(predictions):
            pred_scaled = (pred[0] * scale, pred[1] * scale)
            # Draw dashed line segments
            if i % 2 == 0:
                cv2.line(frame, prev, pred_scaled, color, 1)
            prev = pred_scaled
        # Draw arrow at end of prediction
        if len(predictions) >= 2:
            end = (predictions[-1][0] * scale, predictions[-1][1] * scale)
            cv2.circle(frame, end, 4, color, -1)

def camera_thread():
    global camera_running, current_frame
    
    # Create camera object
    zed = sl.Camera()
    
    # Set initialization parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.VGA  # VGA for max frame rate
    init_params.camera_fps = 100  # 100fps for fast puck tracking
    init_params.depth_mode = sl.DEPTH_MODE.NONE  # Disable depth for better performance
    
    # Open camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Error opening camera: {err}")
        return
    
    print("Camera opened successfully")
    
    # Optimize camera settings for image clarity
    zed.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 8)  # Max sharpness (0-8)
    zed.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 5)   # Slightly boost contrast (0-8)
    zed.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, 5) # Balanced saturation (0-8)
    
    # Create image container
    image = sl.Mat()
    runtime_params = sl.RuntimeParameters()
    
    camera_running = True
    
    while camera_running:
        # Grab frame
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image
            zed.retrieve_image(image, sl.VIEW.LEFT)
            
            # Convert to numpy array
            frame = image.get_data()
            
            # Convert BGRA to BGR for JPEG encoding (ZED returns BGRA format)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            
            # Crop to table/workspace region
            x1, y1, x2, y2 = CROP_REGION
            frame_cropped = frame_bgr[y1:y2, x1:x2]
            
            # Convert to HSV for color detection
            frame_hsv = cv2.cvtColor(frame_cropped, cv2.COLOR_BGR2HSV)
            
            # Detect green puck
            green_center, green_bbox = detect_puck(frame_hsv, GREEN_LOWER, GREEN_UPPER)
            if green_center:
                green_history.append(green_center)
            
            # Detect red puck
            red_center, red_bbox = detect_puck(frame_hsv, RED_LOWER1, RED_UPPER1, RED_LOWER2, RED_UPPER2)
            if red_center:
                red_history.append(red_center)
            
            # Predict trajectories
            green_predictions = predict_trajectory(green_history)
            red_predictions = predict_trajectory(red_history)
            
            # Scale up for display
            h, w = frame_cropped.shape[:2]
            frame_scaled = cv2.resize(frame_cropped, (w * DISPLAY_SCALE, h * DISPLAY_SCALE), 
                                      interpolation=cv2.INTER_LINEAR)
            
            # Draw tracking visualizations
            if green_center and green_bbox:
                draw_tracking(frame_scaled, green_center, green_bbox, green_history, 
                            green_predictions, (0, 255, 0), "GREEN")
            
            if red_center and red_bbox:
                draw_tracking(frame_scaled, red_center, red_bbox, red_history,
                            red_predictions, (0, 0, 255), "RED")
            
            # Update global frame
            with frame_lock:
                current_frame = frame_scaled.copy()
    
    zed.close()
    print("Camera closed")

def generate_frames():
    """Generate frames for streaming"""
    while True:
        with frame_lock:
            if current_frame is None:
                continue
            frame = current_frame.copy()
        
        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ret:
            continue
        
        # Convert to bytes
        frame_bytes = buffer.tobytes()
        
        # Yield frame in HTTP streaming format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    """Home page with embedded video"""
    return """
    <html>
    <head>
        <title>ZED Camera Stream</title>
        <style>
            body {
                background-color: #000;
                margin: 0;
                padding: 20px;
                font-family: Arial, sans-serif;
                color: white;
                text-align: center;
            }
            h1 {
                margin-bottom: 20px;
            }
            img {
                max-width: 100%;
                height: auto;
                border: 2px solid #333;
            }
        </style>
    </head>
    <body>
        <h1>ZED 2i Puck Tracking</h1>
        <img src="/video_feed" alt="Camera Feed">
        <p>VGA @ 100 FPS | Tracking: GREEN and RED pucks</p>
    </body>
    </html>
    """

@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Start camera in background thread
    cam_thread = threading.Thread(target=camera_thread, daemon=True)
    cam_thread.start()
    
    # Wait for camera to initialize
    import time
    time.sleep(2)
    
    print("\n" + "="*50)
    print("ZED Camera Web Stream")
    print("="*50)
    print("Access the stream at:")
    print("  http://<jetson-ip>:5000")
    print("  or")
    print("  http://localhost:5000  (if on Jetson)")
    print("="*50 + "\n")
    
    # Run Flask server
    app.run(host='0.0.0.0', port=5000, threaded=True)
