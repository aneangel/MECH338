#!/usr/bin/env python3
"""
Simple multithreaded PID control to move the end-effector to the puck.

Startup sequence:
    - Open serial
    - Send "HOME" and block until "HOMEXY COMPLETE" is received
    - Start serial writer thread

Thread 1 (ZEDTracker):
    - Grabs frames from ZED
    - Detects red puck & green end-effector (inside CAMERA_BOUNDARY ROI)
    - Draws overlays on the frame
    - Stores latest puck/paddle positions + frame

Thread 2 (main thread):
    - Reads latest puck/paddle positions from tracker
    - Runs PID in motor-error space
    - Updates *target velocity* in MotorController (non-blocking)
    - Displays the latest frame

Thread 3 (Motor writer):
    - At fixed rate, reads latest target velocity and sends "vx vy\n" over serial
    - No command queue, no buffering: MCU always sees the latest setpoint
"""

import time
import threading

import cv2
import numpy as np
import pyzed.sl as sl
import serial   # pyserial


# ===================== CONFIG =====================

# ---- Serial ----
SERIAL_PORT = "/dev/ttyACM1"   # change to your RP2350 port
BAUDRATE = 115200
SERIAL_CMD_PERIOD = 0.5      # seconds between velocity sends (200 Hz)

# ---- PID gains (camera error -> motor velocity units) ----
P_GAIN = 10.0     # proportional
I_GAIN = 0.01     # integral
D_GAIN = 0.5      # derivative

MAX_VELOCITY = 10000.0   # mm/s (or whatever your firmware expects)
MIN_VELOCITY = 40.0    # minimum speed to overcome friction

PIXEL_DEADBAND = 5     # ignore tiny errors (pixels)

# ---- Camera / mapping ----
CAMERA_BOUNDARY = (10, 10, 1280, 720)  # (x_min, y_min, x_max, y_max)

# Map camera error -> motor axes
# Each tuple: (which camera axis to use: 'x' or 'y', sign: +1 or -1)
MOTOR_X_MAP = ('y', 1)   # motor X responds to camera Y
MOTOR_Y_MAP = ('x', -1)  # motor Y responds to camera X (inverted)

# ---- Color thresholds ----
RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 100])
RED_UPPER2 = np.array([180, 255, 255])

GREEN_LOWER = np.array([35, 100, 100])
GREEN_UPPER = np.array([85, 255, 255])

MIN_AREA = 80  # min contour area

# ---- Morphology kernel (reused) ----
MORPH_KERNEL = np.ones((3, 3), np.uint8)


# ===================== HELPERS =====================

class PIDAxis:
    """Simple PID controller for one axis."""
    def __init__(self, kp, ki, kd, i_limit=200.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        # integral term
        self.integral += error * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))

        # derivative term
        d_err = (error - self.prev_error) / dt
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * d_err


class MotorController:
    """
    Serial interface:

      - connect()      : open port
      - home()         : send "HOME", block until "HOMEXY COMPLETE"
      - start_writer() : start async writer thread
      - velocity(vx,vy): update target velocity (non-blocking)

    Writer thread:
      - Runs at SERIAL_CMD_PERIOD
      - Reads latest target vx,vy under a lock
      - Sends 'vx vy\\n' to serial
      - No command queue, so no buffering.
    """
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        # shared target velocity
        self._vx_target = 0.0
        self._vy_target = 0.0
        self._target_lock = threading.Lock()

        self.writer_running = False
        self.writer_thread = None

    def connect(self):
        try:
            print(f"[Motor] Opening {self.port} @ {self.baudrate}...")
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1.0,       # for blocking reads in home()
                write_timeout=None  # avoid long blocking writes
            )
            time.sleep(2.0)  # let RP2350 boot
            print("[Motor] Connected")
            return True
        except Exception as e:
            print(f"[Motor] Failed to open serial: {e}")
            return False

    def _write_line_immediate(self, line: str):
        """Synchronous write, used for HOME before writer thread starts."""
        if not self.ser or not self.ser.is_open:
            return
        try:
            if not line.endswith("\n"):
                line += "\n"
            self.ser.flush()
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            print(f"[Motor] Immediate write error: {e}")

    def home(self, timeout=60.0):
        """
        Send HOME and block until 'HOMEXY COMPLETE' is seen or timeout.
        """
        if not self.ser or not self.ser.is_open:
            print("[Motor] Can't home: serial not open")
            return False

        print("[Motor] Sending HOME command...")
        self._write_line_immediate("HOME")

        start = time.time()
        buffer = b""

        while time.time() - start < timeout:
            if self.ser.in_waiting:
                buffer += self.ser.read(self.ser.in_waiting)
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    text = line.decode("utf-8", errors="ignore").strip()
                    if not text:
                        continue
                    print(f"[Motor] << {text}")
                    if "HOMEXY COMPLETE" in text:
                        print("[Motor] Homing complete")
                        # Clear any remaining RX noise
                        try:
                            self.ser.reset_input_buffer()
                        except Exception:
                            pass
                        return True
            time.sleep(0.01)

        print("[Motor] Homing timeout (no 'HOMEXY COMPLETE')")
        return False

    # ---- writer thread ----

    def start_writer(self):
        """
        Start background writer thread. It sends the latest target
        velocity at fixed rate. Call after homing completes.
        """
        if not self.ser or not self.ser.is_open:
            print("[Motor] Can't start writer: serial not open")
            return False
        if self.writer_thread is not None and self.writer_thread.is_alive():
            return True

        self.writer_running = True
        self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.writer_thread.start()
        print("[Motor] Writer thread started")
        return True

    def _writer_loop(self):
        last_cmd = None
        while self.writer_running:
            #with self._target_lock:
            vx = self._vx_target
            vy = self._vy_target

            # Clamp and deadband here, to keep main thread cheap
            vx = max(-MAX_VELOCITY, min(MAX_VELOCITY, vx))
            vy = max(-MAX_VELOCITY, min(MAX_VELOCITY, vy))

            if abs(vx) < MIN_VELOCITY:
                vx = 0.0
            if abs(vy) < MIN_VELOCITY:
                vy = 0.0

            cmd = f"{vx:.2f} {vy:.2f}"

            if self.ser and self.ser.is_open:
                try:
                    # Optional: avoid spamming identical commands
                    if cmd != last_cmd:
                        if not cmd.endswith("\n"):
                            cmd_to_send = cmd + "\n"
                        else:
                            cmd_to_send = cmd
                        print(f"cmd: {cmd}")
                        self.ser.write(cmd_to_send.encode("utf-8"))
                        last_cmd = cmd
                except Exception as e:
                    print(f"[Motor] Writer error: {e}")

            time.sleep(SERIAL_CMD_PERIOD)

    # ---- public API used by main loop ----

    def velocity(self, vx: float, vy: float):
        """
        Update target velocity. Non-blocking: writer thread will stream it out.
        """
        with self._target_lock:
            self._vx_target = float(vx)
            self._vy_target = float(vy)

    def stop(self):
        self.velocity(0.0, 0.0)

    def close(self):
        # Send stop command immediately (synchronous, bypasses writer thread)
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b"0.00 0.00\n")
                self.ser.flush()
                time.sleep(0.1)  # Give MCU time to process
        except Exception:
            pass

        # Stop writer thread
        self.writer_running = False
        if self.writer_thread is not None:
            self.writer_thread.join(timeout=1.0)

        # Close serial
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("[Motor] Closed")
        except Exception:
            pass


def camera_to_motor(cam_ex, cam_ey):
    """
    Convert camera pixel errors to motor velocities (before PID scaling).
    cam_ex > 0 -> puck is to the RIGHT in the camera.
    cam_ey > 0 -> puck is BELOW in the camera.
    """
    x_axis, x_sign = MOTOR_X_MAP
    y_axis, y_sign = MOTOR_Y_MAP

    motor_ex = x_sign * (cam_ex if x_axis == "x" else cam_ey)
    motor_ey = y_sign * (cam_ex if y_axis == "x" else cam_ey)

    return motor_ex, motor_ey


def detect_red_puck(hsv_roi):
    """
    Detect red puck in HSV ROI image.
    Returns (x, y) in ROI coordinates or None.
    """
    mask1 = cv2.inRange(hsv_roi, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv_roi, RED_LOWER2, RED_UPPER2)
    mask = cv2.bitwise_or(mask1, mask2)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    best_area = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue
        perim = cv2.arcLength(c, True)
        if perim == 0:
            continue
        circ = 4 * np.pi * area / (perim * perim)
        if circ > 0.5 and area > best_area:
            best_area = area
            best_cnt = c

    if best_cnt is not None:
        M = cv2.moments(best_cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
    return None


def detect_green_paddle(hsv_roi):
    """
    Detect green paddle in HSV ROI image.
    Returns (x, y) in ROI coordinates or None.
    """
    mask = cv2.inRange(hsv_roi, GREEN_LOWER, GREEN_UPPER)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    best_area = 0

    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_AREA:
            continue
        perim = cv2.arcLength(c, True)
        if perim == 0:
            continue
        circ = 4 * np.pi * area / (perim * perim)
        if circ > 0.5 and area > best_area:
            best_area = area
            best_cnt = c

    if best_cnt is not None:
        M = cv2.moments(best_cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
    return None


# ===================== ZED TRACKER THREAD =====================

class ZEDTracker:
    """
    Runs in a background thread:
      - grabs frames
      - runs puck & paddle detection inside CAMERA_BOUNDARY ROI
      - draws overlays
      - stores latest positions + frame
    """

    def __init__(self):
        self.zed = None
        self.running = False
        self.thread = None

        self.puck_pos = None
        self.paddle_pos = None
        self.latest_frame = None

        self.lock = threading.Lock()

    def start(self):
        """Open ZED and start tracking thread."""
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30
        init_params.depth_mode = sl.DEPTH_MODE.NONE

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"[ZED] Failed to open camera: {err}")
            return False

        print("[ZED] Opened, starting tracking thread...")
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        """Stop tracking thread and close camera."""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.zed is not None:
            self.zed.close()
            print("[ZED] Closed")

    def _loop(self):
        runtime = sl.RuntimeParameters()
        image = sl.Mat()
        x1, y1, x2, y2 = CAMERA_BOUNDARY

        while self.running:
            if self.zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            self.zed.retrieve_image(image, sl.VIEW.RIGHT)
            frame_full = image.get_data()[:, :, :3]
            frame = frame_full.copy()

            roi = frame[y1:y2, x1:x2]
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            puck_roi = detect_red_puck(hsv_roi)
            paddle_roi = detect_green_paddle(hsv_roi)

            puck_pos = (puck_roi[0] + x1, puck_roi[1] + y1) if puck_roi is not None else None
            paddle_pos = (paddle_roi[0] + x1, paddle_roi[1] + y1) if paddle_roi is not None else None

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)

            if puck_pos is not None:
                cv2.circle(frame, puck_pos, 15, (0, 0, 255), 3)
                cv2.putText(frame, "PUCK", (puck_pos[0] - 20, puck_pos[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            if paddle_pos is not None:
                cv2.circle(frame, paddle_pos, 15, (0, 255, 0), 3)
                cv2.putText(frame, "PADDLE", (paddle_pos[0] - 25, paddle_pos[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if puck_pos is not None and paddle_pos is not None:
                cv2.line(frame, paddle_pos, puck_pos, (255, 0, 255), 2)

            with self.lock:
                self.puck_pos = puck_pos
                self.paddle_pos = paddle_pos
                self.latest_frame = frame

            time.sleep(0.001)

    def get_state(self):
        with self.lock:
            puck = self.puck_pos
            paddle = self.paddle_pos
            frame = self.latest_frame
        return puck, paddle, frame


# ===================== MAIN =====================

def main():
    # ---- Motor ----
    motor = MotorController(SERIAL_PORT, BAUDRATE)
    if not motor.connect():
        return

    # Blocking HOME sequence (no ZED yet)
    if not motor.home(timeout=120.0):
        print("[Main] Homing failed or timed out. Aborting.")
        motor.close()
        return

    # Start async writer thread for velocity streaming
    if not motor.start_writer():
        print("[Main] Failed to start writer thread. Aborting.")
        motor.close()
        return

    # ---- ZED Tracker ----
    tracker = ZEDTracker()
    if not tracker.start():
        motor.close()
        return

    pid_x = PIDAxis(P_GAIN, I_GAIN, D_GAIN)
    pid_y = PIDAxis(P_GAIN, I_GAIN, D_GAIN)

    last_time = time.time()

    print("\n=== Simple Multithreaded PID puck-following ===")
    print("Press 'q' in the window or Ctrl+C in the terminal to quit.\n")

    try:
        while True:
            puck_pos, paddle_pos, frame = tracker.get_state()

            now = time.time()
            dt = now - last_time
            last_time = now

            vx_cmd = 0.0
            vy_cmd = 0.0

            if puck_pos is not None and paddle_pos is not None and dt > 0.0:
                cam_ex = puck_pos[0] - paddle_pos[0]
                cam_ey = puck_pos[1] - paddle_pos[1]

                if abs(cam_ex) < PIXEL_DEADBAND:
                    cam_ex = 0.0
                if abs(cam_ey) < PIXEL_DEADBAND:
                    cam_ey = 0.0

                ex, ey = camera_to_motor(cam_ex, cam_ey)

                vx_cmd = pid_x.update(ex, dt)
                vy_cmd = pid_y.update(ey, dt)
            else:
                pid_x.reset()
                pid_y.reset()

            # Non-blocking: just update target
            #print(f"vcmd: {vx_cmd} {vy_cmd}")
            motor.velocity(vx_cmd, vy_cmd)

            if frame is not None:
                cv2.putText(frame, f"V({vx_cmd:+.1f},{vy_cmd:+.1f})",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 0, 255), 2)
                cv2.imshow("PID Puck Follow (multithreaded)", cv2.resize(frame, (960, 540)))
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break

            time.sleep(0.002)

    except KeyboardInterrupt:
        print("\n[Main] Ctrl+C, stopping...")

    finally:
        motor.stop()
        motor.close()
        tracker.stop()
        cv2.destroyAllWindows()
        print("[Main] Done.")


if __name__ == "__main__":
    main()

