# Air Hockey Defense System - Quick Start Guide

## System Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│  ZED Camera     │         │  Jetson Nano     │         │  XIAO RP2350    │
│  (stream_camera)│ ──────> │  (Defense        │ ──────> │  (Motor         │
│                 │  HTTP   │   Controller)    │  JSON   │   Control)      │
│  - Track puck   │         │  - Predict path  │  USB    │  - CoreXY       │
│  - Kalman       │         │  - Defense logic │         │  - Sensorless   │
│    filter       │         │  - Send commands │         │    homing       │
└─────────────────┘         └──────────────────┘         └─────────────────┘
```

## Hardware Setup

### RP2350 Connections (Motor_Control_JSON.cpp)
- **Motor A (Right):**
  - STEP: D1
  - DIR: D0
  - EN: D4
  - DIAG: D10
  
- **Motor B (Left):**
  - STEP: D5
  - DIR: D2
  - EN: D8
  - DIAG: D9

- **TMC2209 UART:**
  - RX: D7
  - TX: D6

### Wiring
- Both TMC2209 on same UART bus (addresses 0b00 and 0b01)
- DIAG pins for sensorless homing
- 12-24V power to TMC2209 drivers

## Software Setup

### 1. Upload Firmware to RP2350

```bash
cd /home/aneangel/MECH338/arduino
pio run -e seeed_xiao_rp2350 --target upload
```

### 2. Start Camera Stream

```bash
cd /home/aneangel/MECH338/zed
python3 stream_camera.py
```

Access camera at: http://localhost:5000

### 3. Run Defense Controller

```bash
cd /home/aneangel/MECH338/final
python3 hockey_defense_controller.py
```

## JSON Command Protocol

### Python -> RP2350 Commands

```json
// Ping test
{"cmd": "ping"}

// Enable/disable motors
{"cmd": "enable", "state": true}
{"cmd": "enable", "state": false}

// Run sensorless homing calibration
{"cmd": "home"}

// Move to absolute position (mm)
{"cmd": "move", "x": 35.0, "y": 17.5}

// Set velocity (mm/s)
{"cmd": "velocity", "x_vel": 100.0, "y_vel": 50.0}

// Stop all motion
{"cmd": "stop"}

// Get status
{"cmd": "status"}
```

### RP2350 -> Python Responses

```json
{
  "cmd": "ping",
  "status": "ok",
  "message": "pong",
  "x": 35.0,
  "y": 17.5,
  "homed": true,
  "enabled": true
}

// Status response
{
  "cmd": "status",
  "status": "ok",
  "x": 35.0,
  "y": 17.5,
  "x_min": 0.0,
  "x_max": 70.0,
  "y_min": 0.0,
  "y_max": 35.0,
  "homed": true,
  "enabled": true,
  "x_vel": 0.0,
  "y_vel": 0.0
}
```

## Operation Modes

### Manual Control Mode

Commands available in manual mode:
- `right` - Move right at 100 mm/s
- `left` - Move left at 100 mm/s
- `up` - Move up at 100 mm/s
- `down` - Move down at 100 mm/s
- `stop` - Stop all motion
- `center` - Move to center of workspace
- `home` - Run sensorless homing
- `quit` - Exit

### Autonomous Defense Mode

Command: `auto`

The system will:
1. Read puck position from camera stream
2. Predict puck trajectory
3. Calculate intercept point
4. Move paddle to defend goal

Parameters (in `hockey_defense_controller.py`):
- `defense_line_x = 15.0` - Defend at 15mm from left edge
- `reaction_time = 0.3` - Prediction lookahead (seconds)
- `max_speed = 150.0` - Maximum paddle speed (mm/s)

## CoreXY Kinematics

All inverse kinematics calculated on RP2350:

```cpp
// Python sends: (x_vel, y_vel) in mm/s
// RP2350 calculates: (motor_A_steps, motor_B_steps)

CoreXY Formula:
  Motor A = X + Y
  Motor B = X - Y
```

**Boundary Checking:**
- Workspace limits set by sensorless homing
- All commands constrained to valid workspace
- Velocity commands stop at boundaries

## Sensorless Homing Sequence

1. **Find X-min** (both motors same direction)
2. **Find Y-min** (motors opposite directions)
3. **Find X-max** (measure workspace width)
4. **Find Y-max** (measure workspace height)
5. **Return to origin** (X-min, Y-min)

Total time: ~30-45 seconds

## Troubleshooting

### Motors don't move
- Check enable pins (LOW = enabled)
- Verify TMC2209 power supply
- Check UART connection with ping command

### Homing doesn't detect edges
- Adjust `HOME_SGTHRS` (currently 5, range 0-255)
- Lower = more sensitive
- Check DIAG pin connections
- Ensure sufficient motor current (rms_current=1500)

### Camera tracking not working
- Verify ZED camera connected
- Check stream at http://localhost:5000
- Adjust HSV color ranges in stream_camera.py
- Check tracking_data endpoint: http://localhost:5000/tracking_data

### Paddle not intercepting puck
- Tune `reaction_time` (0.3s default)
- Adjust `max_speed` (150 mm/s default)
- Check coordinate mapping in predict_intercept()
- Verify workspace calibration matches camera view

## Performance Tuning

### Increase Speed
```cpp
// In Motor_Control_JSON.cpp
const uint16_t VELOCITY_UPDATE_MS = 10;  // Faster updates (was 20)
```

### Smoother Motion
```python
# In hockey_defense_controller.py
k_p = 3.0  # Lower gain = smoother (was 5.0)
```

### Better Prediction
```python
# In hockey_defense_controller.py
lookahead_time = 0.5  # Longer lookahead (was 0.3)
```

## Testing Sequence

1. **Test Motors:**
   ```
   >>> right
   >>> stop
   >>> left
   >>> stop
   ```

2. **Test Homing:**
   ```
   >>> home
   (Wait for completion)
   ```

3. **Test Positioning:**
   ```
   >>> center
   ```

4. **Test Camera:**
   - Open browser: http://localhost:5000
   - Verify puck tracking (red object)
   - Check tracking_data: http://localhost:5000/tracking_data

5. **Test Defense:**
   ```
   >>> auto
   (Move puck in camera view)
   >>> stop
   ```

## Safety Notes

- Always home before autonomous mode
- Emergency stop: Ctrl+C or `stop` command
- Workspace boundaries enforced by firmware
- Manual disable: `{"cmd": "enable", "state": false}`

## File Locations

- Firmware: `/home/aneangel/MECH338/arduino/src/Motor_Control_JSON.cpp`
- Defense Controller: `/home/aneangel/MECH338/final/hockey_defense_controller.py`
- Camera Stream: `/home/aneangel/MECH338/zed/stream_camera.py`
- PlatformIO Config: `/home/aneangel/MECH338/arduino/platformio.ini`
