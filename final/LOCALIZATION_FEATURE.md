# Vision-Based Localization Feature

## Overview
This feature allows the air hockey defense system to recover its position using camera tracking of the green paddle (end-effector) without needing to run the full sensorless homing sequence.

## Use Case
When the motors stop in an arbitrary position (e.g., after a power loss, emergency stop, or manual intervention), the system can use the camera to determine where the paddle is and update the controller's internal position estimate.

## How It Works

### Hardware
- **Green circle marker** attached to the paddle (end-effector)
- **ZED 2i camera** tracking both red puck and green paddle
- **HSV color detection** to locate the green circle in the camera frame

### Coordinate Transformation
1. Camera detects green circle position in pixels (cropped region: 315×188)
2. Kalman filter smooths the position estimate
3. Position is converted to workspace coordinates:
   - Camera: (0-315, 0-188) pixels
   - Workspace: (0-70, 0-35) mm

### Firmware Command
New `localize` command added to Motor_Control.cpp:

```json
{
  "cmd": "localize",
  "x": 35.5,
  "y": 17.5
}
```

**Response:**
```json
{
  "cmd": "localize",
  "status": "ok"
}
```

**Error conditions:**
- Position out of workspace bounds
- Missing x or y parameters
- Workspace not yet homed (boundaries unknown)

### Python Interface

```python
# Get paddle position from camera
paddle_pos = tracker.get_paddle_position()  # Returns (cam_x, cam_y) in pixels

# Convert to workspace coordinates
workspace_x, workspace_y = tracker.camera_to_workspace(cam_x, cam_y)

# Update motor controller position
motor.localize_from_vision(workspace_x, workspace_y)
```

## Usage

### Manual Command
```bash
python3 hockey_defense_controller.py
```

Then use the `localize` command:
```
>>> localize
Looking for green paddle in camera view...
  Camera position: (157, 94) pixels
  Workspace position: (35.0, 17.5) mm
✓ Position localized to (35.0, 17.5) mm
✓ Position recovered! You can now use velocity commands.
```

### Requirements
1. Workspace must have been homed **at least once** to establish boundaries (x_min, x_max, y_min, y_max)
2. Green paddle must be **visible** in the camera view
3. Camera must be **initialized and tracking**

### Advantages Over Full Homing
- **No motor movement required** - motors can stay disabled
- **Faster** - instant position recovery vs. ~60 seconds for full homing
- **Safer** - no risk of collision during homing moves
- **Flexible** - works from any position within the workspace

### Limitations
- Requires initial homing to establish workspace boundaries
- Accuracy depends on camera calibration and green circle visibility
- Cannot detect if workspace boundaries have changed physically
- Assumes camera view remains aligned with workspace

## Implementation Details

### Firmware Changes (`Motor_Control.cpp`)
- Added `handleLocalizeCommand()` function
- Validates position is within known workspace bounds
- Stops any ongoing motion when position is updated
- Wired into JSON command processor

### Python Changes (`hockey_defense_controller.py`)
- Added `GREEN_LOWER` and `GREEN_UPPER` HSV thresholds
- Added `kf_paddle` Kalman filter for green circle tracking
- Added `_detect_paddle()` method using same circularity check as puck
- Added `get_paddle_position()` getter method
- Added `camera_to_workspace()` coordinate converter
- Added `localize_from_vision()` method to MotorController
- Added `localize` command to manual control interface

### Tracking Performance
- **Update rate:** 60 FPS (ZED camera)
- **Kalman filter:** 4-state (x, y, vx, vy) with lower measurement noise than puck
- **Detection:** Circularity check (>0.6) with minimum area threshold (80 pixels)

## Testing Procedure

1. **Run homing once** to establish boundaries:
   ```bash
   python3 hockey_defense_controller.py
   >>> home
   ```

2. **Move paddle manually** (with motors disabled or after emergency stop)

3. **Run localize** to recover position:
   ```bash
   >>> localize
   ```

4. **Verify position** using status command or by issuing velocity commands

## Future Enhancements
- Automatic localization on startup if green paddle is visible
- Continuous position correction during operation (sensor fusion)
- Multiple reference points for improved accuracy
- Adaptive workspace boundary detection
