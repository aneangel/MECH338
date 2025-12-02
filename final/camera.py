"""
Air Hockey Robot Vision System with Motor Control

Integrates USB webcam vision with motor controller for air hockey robot.
Detects black objects (puck) and provides real-time tracking information.

Usage:
    python camera.py
"""

import cv2
import time
import os
import numpy as np
import sys
from computation import TrajectoryCalculator

# Import motor controller
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from sense import MotorController
except ImportError:
    print("Warning: Could not import MotorController from sense.py")
    MotorController = None

# --- Configuration ---
CAMERA_INDEX = 0
WIDTH = 1280
HEIGHT = 720
FPS = 30

# Black object detection parameters
BLACK_THRESHOLD_LOW = np.array([0, 0, 0], dtype=np.uint8)
BLACK_THRESHOLD_HIGH = np.array([180, 255, 50], dtype=np.uint8)  # HSV thresholds
MIN_CONTOUR_AREA = 100  # Minimum area to consider as valid detection

# Program Mode Configuration
SAVE_INTERVAL = 2.0
OUTPUT_DIR = "camera_captures"
# --- End Configuration ---

def has_display():
    """
    Check if display is available for showing video feed.
    
    Returns:
        bool: True if display is available, False if headless
    """
    try:
        display = os.environ.get('DISPLAY')
        if display and display.strip():
            return True
        
        wayland = os.environ.get('WAYLAND_DISPLAY')
        if wayland and wayland.strip():
            return True
            
        return False
    except:
        return False

def calculate_focus_score(image):
    """
    Calculate focus score using Laplacian variance.
    Higher values indicate better focus.
    
    Args:
        image: OpenCV image (BGR format)
        
    Returns:
        float: Focus score (higher = better focus)
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    return laplacian_var

def get_focus_quality_rating(score):
    """
    Convert focus score to human-readable rating.
    
    Args:
        score: Focus score from calculate_focus_score()
        
    Returns:
        str: Focus quality description
    """
    if score > 1000:
        return "EXCELLENT"
    elif score > 500:
        return "GOOD"
    elif score > 200:
        return "FAIR"
    elif score > 100:
        return "POOR"
    else:
        return "VERY POOR"

def detect_black_objects(frame):
    """
    Detect black objects in the frame (for puck detection).
    
    Args:
        frame: OpenCV BGR image
        
    Returns:
        tuple: (detections_list, annotated_frame)
            detections_list: List of dict with 'center', 'area', 'bbox'
            annotated_frame: Frame with detection visualizations
    """
    # Convert to HSV for better color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create mask for black objects
    mask = cv2.inRange(hsv, BLACK_THRESHOLD_LOW, BLACK_THRESHOLD_HIGH)
    
    # Apply morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    detections = []
    annotated_frame = frame.copy()
    
    for contour in contours:
        area = cv2.contourArea(contour)
        
        # Filter by minimum area
        if area > MIN_CONTOUR_AREA:
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate center
            center_x = x + w // 2
            center_y = y + h // 2
            
            # Store detection info
            detection = {
                'center': (center_x, center_y),
                'area': area,
                'bbox': (x, y, w, h)
            }
            detections.append(detection)
            
            # Draw on frame
            cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.putText(annotated_frame, f"Area: {int(area)}", 
                       (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return detections, annotated_frame

def focus_calibration_mode(cap):
    """
    Interactive focus calibration mode.
    """
    display_available = has_display()
    
    print("\n" + "="*60)
    print("FOCUS CALIBRATION MODE")
    print("="*60)
    print("Instructions:")
    print("- Manually adjust your camera focus while watching the scores")
    if display_available:
        print("- Live video feed will be displayed")
        print("- Press 'q' in video window OR terminal to exit")
    else:
        print("- Running in headless mode (no video display)")
        print("- Press 'q' to return to main menu")
    print("- Focus score above 500 is GOOD, above 1000 is EXCELLENT")
    print("- Press 's' to save a test image")
    print("="*60)
    
    best_score = 0
    frame_count = 0
    window_name = "Focus Calibration - Live Feed"
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Error: Failed to retrieve frame.")
                time.sleep(0.01)
                continue
            
            frame_count += 1
            
            if frame_count % 5 == 0:
                focus_score = calculate_focus_score(frame)
                quality = get_focus_quality_rating(focus_score)
                
                if focus_score > best_score:
                    best_score = focus_score
                
                if display_available:
                    cv2.putText(frame, f"Focus: {focus_score:.1f} ({quality})", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Best: {best_score:.1f}", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.putText(frame, "Press 'q' to quit, 's' to save", 
                               (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                else:
                    print(f"\rFocus Score: {focus_score:7.1f} | Quality: {quality:10} | Best: {best_score:7.1f}", end="", flush=True)
            
            if display_available:
                cv2.imshow(window_name, frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\nReturning to main menu...")
                break
            elif key == ord('s'):
                focus_score = calculate_focus_score(frame)
                quality = get_focus_quality_rating(focus_score)
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"focus_test_{timestamp}_score_{focus_score:.1f}.jpg"
                cv2.imwrite(filename, frame)
                print(f"\nSaved test image: {filename}")
            
            time.sleep(0.033)
            
    except KeyboardInterrupt:
        print("\nFocus calibration interrupted by user.")
    
    finally:
        if display_available:
            cv2.destroyWindow(window_name)

def take_test_shot(cap):
    """
    Take a single test shot and analyze it.
    """
    print("\nTaking test shot...")
    
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        return
    
    focus_score = calculate_focus_score(frame)
    quality = get_focus_quality_rating(focus_score)
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"test_shot_{timestamp}.jpg"
    cv2.imwrite(filename, frame)
    
    print(f"Test shot saved: {filename}")
    print(f"Focus Score: {focus_score:.1f}")
    print(f"Focus Quality: {quality}")
    print(f"Resolution: {frame.shape[1]}x{frame.shape[0]}")

def show_camera_info(cap):
    """
    Display detailed camera information.
    """
    print("\n" + "="*40)
    print("CAMERA INFORMATION")
    print("="*40)
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
    contrast = cap.get(cv2.CAP_PROP_CONTRAST)
    saturation = cap.get(cv2.CAP_PROP_SATURATION)
    
    print(f"Resolution: {width}x{height}")
    print(f"FPS: {fps}")
    print(f"Brightness: {brightness}")
    print(f"Contrast: {contrast}")
    print(f"Saturation: {saturation}")
    print(f"Display Available: {has_display()}")
    print(f"Motor Controller: {'Available' if MotorController else 'Not Available'}")
    print("="*40)

def live_video_preview(cap):
    """
    Show live video preview with black object detection.
    """
    if not has_display():
        print("Error: No display available for video preview.")
        return
    
    print("\n" + "="*60)
    print("LIVE VIDEO PREVIEW WITH BLACK OBJECT DETECTION")
    print("="*60)
    print("Showing live camera feed with puck detection...")
    print("Press 'q' to return to menu")
    print("Press 'f' to toggle focus score overlay")
    print("Press 's' to save current frame")
    print("="*60)
    
    window_name = "Air Hockey Robot - Live Detection"
    show_focus = False
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Error: Failed to retrieve frame.")
                time.sleep(0.01)
                continue
            
            frame_count += 1
            
            # Detect black objects
            detections, annotated_frame = detect_black_objects(frame)
            
            # Print detections to terminal
            if detections:
                print(f"\r[DETECTION] Found {len(detections)} black object(s): ", end="")
                for i, det in enumerate(detections):
                    print(f"Obj{i+1}@({det['center'][0]},{det['center'][1]}) Area={int(det['area'])} | ", end="")
                print("", flush=True)
            
            # Add focus overlay if enabled
            if show_focus and frame_count % 5 == 0:
                focus_score = calculate_focus_score(frame)
                quality = get_focus_quality_rating(focus_score)
                cv2.putText(annotated_frame, f"Focus: {focus_score:.1f} ({quality})", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Add detection count
            cv2.putText(annotated_frame, f"Detections: {len(detections)}", 
                       (10, annotated_frame.shape[0] - 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(annotated_frame, "Press 'q' to quit", 
                       (10, annotated_frame.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(annotated_frame, "Press 'f' for focus, 's' to save", 
                       (10, annotated_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow(window_name, annotated_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('f'):
                show_focus = not show_focus
                print(f"\nFocus overlay: {'ON' if show_focus else 'OFF'}")
            elif key == ord('s'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"detection_{timestamp}.jpg"
                cv2.imwrite(filename, annotated_frame)
                print(f"\nSaved frame: {filename} ({len(detections)} detections)")
    
    except KeyboardInterrupt:
        print("\nLive preview interrupted by user.")
    
    finally:
        cv2.destroyWindow(window_name)
        print("Live preview closed.")

def testing_and_calibration_menu(cap):
    """
    Testing and calibration submenu.
    """
    while True:
        display_available = has_display()
        print("\n" + "="*50)
        print("TESTING AND CALIBRATION MENU")
        print("="*50)
        print(f"Display Status: {'Available' if display_available else 'Headless Mode'}")
        print("="*50)
        print("1. Focus Calibration")
        print("2. Take Test Shot")
        print("3. Camera Information")
        print("4. Live Video with Detection" + (" (Display Available)" if display_available else " (Not Available)"))
        print("5. Back to Main Menu")
        print("="*50)
        
        choice = input("Enter your choice (1-5): ").strip()
        
        if choice == '1':
            focus_calibration_mode(cap)
        elif choice == '2':
            take_test_shot(cap)
        elif choice == '3':
            show_camera_info(cap)
        elif choice == '4':
            if display_available:
                live_video_preview(cap)
            else:
                print("Live video preview requires a display.")
        elif choice == '5':
            break
        else:
            print("Invalid choice. Please enter 1-5.")

def run_program_mode(cap, motor_controller=None):
    """
    Main program mode - black object detection with motor control integration.
    
    Args:
        cap: OpenCV VideoCapture object
        motor_controller: MotorController instance (optional)
    """
    display_available = has_display()
    
    # Initialize trajectory calculator
    trajectory_calc = TrajectoryCalculator(motor_controller)
    
    print("\n" + "="*60)
    print("AIR HOCKEY ROBOT - DETECTION MODE")
    print("="*60)
    print(f"Display Mode: {'Live Video + Detection' if display_available else 'Headless Detection'}")
    print(f"Motor Controller: {'Connected' if motor_controller else 'Not Connected'}")
    print(f"Trajectory Calculator: Initialized")
    print(f"Detection: Black objects (puck tracking)")
    
    if display_available:
        print("Live detection feed will be displayed")
        print("Press 'q' in video window OR Ctrl+C to stop")
    else:
        print("Press Ctrl+C to stop and return to menu")
    
    print("="*60)
    
    window_name = "Air Hockey Robot - Puck Detection"
    
    # ===== MEASURE ACTUAL CAMERA FPS =====
    print("\nMeasuring camera FPS...")
    test_frames = 30
    start_time = time.time()
    for _ in range(test_frames):
        ret, _ = cap.read()
        if not ret:
            break
    measured_fps = test_frames / (time.time() - start_time)
    print(f"Camera FPS: {measured_fps:.1f}")
    
    # ===== CALCULATE OPTIMAL FRAME SKIP =====
    TARGET_PROCESSING_FPS = 10  # Process at 10 FPS for good tracking
    
    if measured_fps < 5:
        PROCESS_EVERY_N_FRAMES = 1  # Process all frames (camera is slow)
        print("Camera is slow - processing every frame")
    elif measured_fps < 15:
        PROCESS_EVERY_N_FRAMES = 1
        print("Processing every frame")
    elif measured_fps < 25:
        PROCESS_EVERY_N_FRAMES = 2  # Process every 2nd frame
        print(f"Processing every 2nd frame (~{measured_fps/2:.1f} FPS)")
    else:
        PROCESS_EVERY_N_FRAMES = int(measured_fps / TARGET_PROCESSING_FPS)
        print(f"Processing every {PROCESS_EVERY_N_FRAMES} frames (~{measured_fps/PROCESS_EVERY_N_FRAMES:.1f} FPS)")
    
    frame_count = 0
    detection_count = 0
    interception_count = 0
    
    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("Error: Failed to retrieve frame.")
                time.sleep(0.01)
                continue

            frame_count += 1
            
            # Detect black objects
            detections, annotated_frame = detect_black_objects(frame)
            
            # Process detections with trajectory calculator
            if detections:
                detection_count += 1
                
                # Use largest detection (most likely the puck)
                largest_detection = max(detections, key=lambda d: d['area'])
                
                print(f"\n[FRAME {frame_count}] PUCK DETECTED:")
                x, y = largest_detection['center']
                area = largest_detection['area']
                print(f"  Position: ({x}, {y}) px, Area: {int(area)} px²")
                
                # Process with trajectory calculator
                result = trajectory_calc.process_detection(largest_detection)
                
                if result:
                    interception_count += 1
                    print(f"  Puck Position: ({result['puck_position'][0]:.1f}, {result['puck_position'][1]:.1f}) mm")
                    print(f"  Puck Velocity: ({result['puck_velocity'][0]:.1f}, {result['puck_velocity'][1]:.1f}) mm/s")
                    print(f"  Intercept Point: ({result['intercept_point'][0]:.1f}, {result['intercept_point'][1]:.1f}) mm")
                    print(f"  Time to Intercept: {result['time_to_intercept']:.3f} s")
                    print(f"  Command Sent: {result['command_sent']}")
            
            # Show video if display available
            if display_available:
                # Add frame info overlay
                cv2.putText(annotated_frame, f"Frame: {frame_count} | Detections: {detection_count} | Interceptions: {interception_count}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(annotated_frame, f"Objects in View: {len(detections)}", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                # Show robot status
                status = trajectory_calc.get_status()
                if status['robot_position']:
                    cv2.putText(annotated_frame, f"Robot: ({status['robot_position'][0]:.0f}, {status['robot_position'][1]:.0f}) mm", 
                               (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                
                cv2.putText(annotated_frame, "Press 'q' to quit", 
                           (10, annotated_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow(window_name, annotated_frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print(f"\nProgram stopped. Frames: {frame_count}, Detections: {detection_count}, Interceptions: {interception_count}")
                    break
            
            time.sleep(0.01)  # Small delay
            
    except KeyboardInterrupt:
        print(f"\nProgram stopped. Frames: {frame_count}, Detections: {detection_count}, Interceptions: {interception_count}")
    
    finally:
        trajectory_calc.stop_motors()
        if display_available:
            cv2.destroyWindow(window_name)

def main_menu():
    """
    Main CLI menu interface.
    """
    display_available = has_display()
    print("\n" + "="*60)
    print("AIR HOCKEY ROBOT VISION SYSTEM")
    print("Jetson Orin Nano - Black Object Detection")
    print("="*60)
    print(f"Display: {'Available' if display_available else 'Headless Mode'}")
    print(f"Motor Control: {'Available' if MotorController else 'Not Available'}")
    print("="*60)
    print("1. Testing and Calibration")
    print("2. Run Detection Mode")
    print("3. Exit")
    print("="*60)

def initialize_camera():
    """
    Initialize and configure the camera.
    
    Returns:
        cv2.VideoCapture: Configured camera object or None if failed
    """
    print(f"Attempting to open camera with index: {CAMERA_INDEX}")
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    if not cap.isOpened():
        print(f"FATAL ERROR: Could not open camera with index {CAMERA_INDEX}.")
        return None

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print("-" * 40)
    print("Camera initialized successfully.")
    print(f"Resolution: {actual_width}x{actual_height}")
    print(f"Display: {has_display()}")
    print("-" * 40)
    
    return cap

def main():
    """
    Main program entry point with CLI interface.
    """
    # Initialize camera
    cap = initialize_camera()
    if cap is None:
        sys.exit(1)
    
    # Initialize motor controller (optional)
    motor_controller = None
    if MotorController:
        print("\nAttempting to connect to motor controller...")
        motor_controller = MotorController(port='/dev/ttyACM0')
        if motor_controller.connect():
            print("Motor controller connected successfully!")
        else:
            print("Motor controller connection failed. Continuing without motors.")
            motor_controller = None
    
    try:
        while True:
            main_menu()
            choice = input("Enter your choice (1-3): ").strip()
            
            if choice == '1':
                testing_and_calibration_menu(cap)
            elif choice == '2':
                run_program_mode(cap, motor_controller)
            elif choice == '3':
                print("Shutting down vision system...")
                break
            else:
                print("Invalid choice. Please enter 1-3.")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    
    finally:
        # Clean up resources
        if cap:
            cap.release()
        if motor_controller:
            motor_controller.disconnect()
        cv2.destroyAllWindows()
        print("Resources released. Goodbye!")


if __name__ == '__main__':
    main()