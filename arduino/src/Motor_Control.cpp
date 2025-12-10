#include <Arduino.h>
#include <TMCStepper.h>
#include <math.h>
#include <pico/mutex.h>

// ====================== USER CONFIG ======================

#define R_SENSE        0.11f

// UART bus (XIAO RP2350)
#define TMC_UART_RX    D7
#define TMC_UART_TX    D6

// CoreXY motor A
#define A_STEP_PIN     D1
#define A_DIR_PIN      D0
#define A_EN_PIN       D4

// CoreXY motor B
#define B_STEP_PIN     D5
#define B_DIR_PIN      D2
#define B_EN_PIN       D8

// DIAG pins for sensorless homing
#define A_DIAG_PIN     D10
#define B_DIAG_PIN     D9

// CoreXY "negative" directions (from your original config)
#define X_NEG_A_DIR    HIGH
#define X_NEG_B_DIR    HIGH
#define Y_NEG_A_DIR    HIGH
#define Y_NEG_B_DIR    LOW

// CoreXY motor directions (positive direction reference) for velocity mode
const uint8_t A_DIR_POS = HIGH;
const uint8_t B_DIR_POS = HIGH;

// Motion parameters
const float STEPS_PER_MM   = 160.0f;   // Adjust for your setup
const float MAX_VELOCITY   = 6000.0f;  // mm/s max speed (limit for commands)
const float MAX_ACCEL      = 60000.0f; // mm/s^2 max accel

// Velocity control timing
const uint16_t VELOCITY_UPDATE_MS = 20;    // 50 Hz for more responsive tracking

// Sensorless homing parameters
const uint16_t HOMING_STEP_DELAY_US  = 800;    // slow & safe toward wall
const uint16_t CENTER_STEP_DELAY_US  = 50;     // faster centering move
const uint32_t HOMING_MAX_STEPS      = 200000; // failsafe limit
const float    HOMING_BACKOFF_MM     = 10.0f;  // backoff distance after stall

// Workspace bounds (effective play area after homing)
const float X_MIN   = 0.0f;
const float X_MAX   = 300.0f;
const float Y_MIN   = 0.0f;
const float Y_MAX   = 230.0f;

// Center of workspace
const float X_CENTER = (X_MIN + X_MAX) * 0.5f;   // 150 mm
const float Y_CENTER = (Y_MIN + Y_MAX) * 0.5f;   // 140 mm

// Command timeout (deadman)
unsigned long last_cmd_time = 0;  // ms, time of last valid velocity command

// =================== GLOBAL STATE ========================

// TMC2209 drivers
TMC2209Stepper drvA(&Serial1, R_SENSE, 0b00);
TMC2209Stepper drvB(&Serial1, R_SENSE, 0b01);

bool drivers_ok      = false;
bool motors_enabled  = false;

// Homing state
bool homed_x = false;
bool homed_y = false;

// Current position estimate (mm, just integrated from velocity)
float cur_x_mm = 0.0f;
float cur_y_mm = 0.0f;

// Velocity control (used by Core 1)
float target_x_vel  = 0.0f;  // mm/s - commanded velocity
float target_y_vel  = 0.0f;  // mm/s
float current_x_vel = 0.0f;  // mm/s - ramped velocity
float current_y_vel = 0.0f;  // mm/s

unsigned long last_move_time = 0;

// ================= DUAL-CORE COMMUNICATION ================

// Mutex for thread-safe velocity updates between cores
mutex_t velocity_mutex;

// Shared velocity target (written by Core 0, read by Core 1)
volatile float shared_target_x_vel = 0.0f;
volatile float shared_target_y_vel = 0.0f;
volatile bool  new_velocity_available = false;

// Abort flag: Core 0 sets this to interrupt Core 1's step loop
volatile bool abort_current_motion = false;

// Flag to force immediate velocity update (bypass timing gate)
volatile bool force_velocity_update = false;

// Flag to block Core 1 during homing (Core 0 controls motors directly)
volatile bool is_homing = false;

// ================= TMC CONFIGURATION ====================

void config_tmc_driver(TMC2209Stepper &drv) {
  drv.begin();
  drv.pdn_disable(true);
  drv.I_scale_analog(false);
  drv.toff(4);
  drv.blank_time(24);
  drv.microsteps(16);
  drv.rms_current(1500);
  drv.en_spreadCycle(false);
  drv.pwm_autoscale(true);
  drv.TCOOLTHRS(0xFFFFF);
  drv.SGTHRS(22);   // sensorless homing threshold - tune as needed
  drv.shaft(false);
}

void check_tmc_connections() {
  uint8_t connA = drvA.test_connection();
  uint8_t connB = drvB.test_connection();
  drivers_ok = (connA == 0 && connB == 0);
}

// ================= MOTION HELPERS =======================

inline bool stall_A() {
  return digitalRead(A_DIAG_PIN) == HIGH;   // invert if needed
}

inline bool stall_B() {
  return digitalRead(B_DIAG_PIN) == HIGH;   // invert if needed
}

// Maximum steps per update to limit blocking time
// 1000 steps * 50us = 50ms max blocking time per update
const int32_t MAX_STEPS_PER_UPDATE = 1000;

// Bresenham-style stepping for CoreXY A/B motors
// Now checks abort_current_motion flag to allow early exit
// Returns number of steps actually executed (for position tracking)
int32_t step_corexy_AB_delta(int32_t dA, int32_t dB, uint16_t step_delay_us) {
  int32_t stepsA = abs(dA);
  int32_t stepsB = abs(dB);
  int32_t maxSteps = max(stepsA, stepsB);

  if (maxSteps == 0) return 0;

  // Cap steps to limit blocking time
  int32_t stepsToExecute = min(maxSteps, MAX_STEPS_PER_UPDATE);

  uint8_t a_dir = (dA >= 0) ? A_DIR_POS : !A_DIR_POS;
  uint8_t b_dir = (dB >= 0) ? B_DIR_POS : !B_DIR_POS;

  digitalWrite(A_DIR_PIN, a_dir);
  digitalWrite(B_DIR_PIN, b_dir);
  delayMicroseconds(20);

  float incA = (stepsA > 0) ? (float)stepsA / maxSteps : 0.0f;
  float incB = (stepsB > 0) ? (float)stepsB / maxSteps : 0.0f;
  float accA = 0.0f;
  float accB = 0.0f;

  int32_t stepsExecuted = 0;

  for (int32_t i = 0; i < stepsToExecute; i++) {
    // Check for abort every 10 steps (~0.5ms at 50us delay)
    // This allows new velocity commands to interrupt quickly
    if ((i % 10) == 0 && abort_current_motion) {
      return stepsExecuted;  // Exit immediately, new command waiting
    }

    accA += incA;
    accB += incB;

    bool doA = false;
    bool doB = false;

    if (accA >= 1.0f) {
      accA -= 1.0f;
      doA = true;
    }
    if (accB >= 1.0f) {
      accB -= 1.0f;
      doB = true;
    }

    if (doA || doB) {
      if (doA) digitalWrite(A_STEP_PIN, HIGH);
      if (doB) digitalWrite(B_STEP_PIN, HIGH);
      delayMicroseconds(3);
      if (doA) digitalWrite(A_STEP_PIN, LOW);
      if (doB) digitalWrite(B_STEP_PIN, LOW);
      delayMicroseconds(step_delay_us);
      stepsExecuted++;
    }
  }

  return stepsExecuted;
}

// ================= SENSORLESS HOMING =====================

// Home X- using sensorless stall detection, backoff 10mm, then go to X center
void home_sensorless_X_neg() {
  is_homing = true;  // Block Core 1 from motor control
  Serial.println("HOMEX (sensorless) START");

  // Stop velocity mode while homing
  target_x_vel  = 0.0f;
  current_x_vel = 0.0f;

  // 1) Move toward X- wall
  digitalWrite(A_DIR_PIN, X_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, X_NEG_B_DIR);
  delayMicroseconds(20);

  uint32_t steps = 0;
  bool stalled = false;

  while (steps < HOMING_MAX_STEPS) {
    digitalWrite(A_STEP_PIN, HIGH);
    digitalWrite(B_STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(A_STEP_PIN, LOW);
    digitalWrite(B_STEP_PIN, LOW);
    delayMicroseconds(HOMING_STEP_DELAY_US);

    steps++;

    if (stall_A() || stall_B()) {
      stalled = true;
      break;
    }
  }

  if (!stalled) {
    Serial.println("HOMEX ABORT (max steps)");
    return;
  }

  Serial.println("HOMEX stall detected, backing off...");

  // 2) Back off 10mm away from wall
  uint32_t backoff_steps = (uint32_t)lroundf(HOMING_BACKOFF_MM * STEPS_PER_MM);

  digitalWrite(A_DIR_PIN, !X_NEG_A_DIR); // away from wall
  digitalWrite(B_DIR_PIN, !X_NEG_B_DIR);
  delayMicroseconds(20);

  for (uint32_t i = 0; i < backoff_steps; ++i) {
    digitalWrite(A_STEP_PIN, HIGH);
    digitalWrite(B_STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(A_STEP_PIN, LOW);
    digitalWrite(B_STEP_PIN, LOW);
    delayMicroseconds(HOMING_STEP_DELAY_US);
  }

  // Now we're ~10mm inside field from X- wall.
  // 3) Move to geometric center (150mm from wall):
  float extra_to_center_mm = X_CENTER - HOMING_BACKOFF_MM;  // 150 - 10 = 140mm

  uint32_t center_steps = (uint32_t)lroundf(extra_to_center_mm * STEPS_PER_MM);

  Serial.println("HOMEX: moving to X center...");
  // Same direction as backoff (away from wall)
  digitalWrite(A_DIR_PIN, !X_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, !X_NEG_B_DIR);
  delayMicroseconds(20);

  for (uint32_t i = 0; i < center_steps; ++i) {
    digitalWrite(A_STEP_PIN, HIGH);
    digitalWrite(B_STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(A_STEP_PIN, LOW);
    digitalWrite(B_STEP_PIN, LOW);
    delayMicroseconds(CENTER_STEP_DELAY_US);
  }

  // Logical position at center
  cur_x_mm = X_CENTER;
  homed_x  = true;

  Serial.println("HOMEX DONE -> at X center");

  target_x_vel  = 0.0f;
  current_x_vel = 0.0f;
  last_cmd_time = millis();
  is_homing = false;  // Allow Core 1 to resume
}

// Home Y- using sensorless stall detection, backoff 10mm, then go to Y center
void home_sensorless_Y_neg() {
  is_homing = true;  // Block Core 1 from motor control
  Serial.println("HOMEY (sensorless) START");

  // Stop velocity mode while homing
  target_y_vel  = 0.0f;
  current_y_vel = 0.0f;

  // 1) Move toward Y- wall
  digitalWrite(A_DIR_PIN, Y_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, Y_NEG_B_DIR);
  delayMicroseconds(20);

  uint32_t steps = 0;
  bool stalled = false;

  while (steps < HOMING_MAX_STEPS) {
    digitalWrite(A_STEP_PIN, HIGH);
    digitalWrite(B_STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(A_STEP_PIN, LOW);
    digitalWrite(B_STEP_PIN, LOW);
    delayMicroseconds(HOMING_STEP_DELAY_US);

    steps++;

    if (stall_A() || stall_B()) {
      stalled = true;
      break;
    }
  }

  if (!stalled) {
    Serial.println("HOMEY ABORT (max steps)");
    return;
  }

  Serial.println("HOMEY stall detected, backing off...");

  // 2) Back off 10mm away from wall
  uint32_t backoff_steps = (uint32_t)lroundf(HOMING_BACKOFF_MM * STEPS_PER_MM);

  digitalWrite(A_DIR_PIN, !Y_NEG_A_DIR); // away from wall
  digitalWrite(B_DIR_PIN, !Y_NEG_B_DIR);
  delayMicroseconds(20);

  for (uint32_t i = 0; i < backoff_steps; ++i) {
    digitalWrite(A_STEP_PIN, HIGH);
    digitalWrite(B_STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(A_STEP_PIN, LOW);
    digitalWrite(B_STEP_PIN, LOW);
    delayMicroseconds(HOMING_STEP_DELAY_US);
  }

  // Now we're ~10mm inside field from Y- wall.
  // 3) Move to geometric center:
  float extra_to_center_mm = Y_CENTER - HOMING_BACKOFF_MM;  // e.g. 140 - 10 = 130mm

  uint32_t center_steps = (uint32_t)lroundf(extra_to_center_mm * STEPS_PER_MM);

  Serial.println("HOMEY: moving to Y center...");
  // Same direction as backoff (away from wall)
  digitalWrite(A_DIR_PIN, !Y_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, !Y_NEG_B_DIR);
  delayMicroseconds(20);

  for (uint32_t i = 0; i < center_steps; ++i) {
    digitalWrite(A_STEP_PIN, HIGH);
    digitalWrite(B_STEP_PIN, HIGH);
    delayMicroseconds(3);
    digitalWrite(A_STEP_PIN, LOW);
    digitalWrite(B_STEP_PIN, LOW);
    delayMicroseconds(CENTER_STEP_DELAY_US);
  }

  // Logical position at center
  cur_y_mm = Y_CENTER;
  homed_y  = true;

  Serial.println("HOMEY DONE -> at Y center");

  target_y_vel  = 0.0f;
  current_y_vel = 0.0f;
  last_cmd_time = millis();
  is_homing = false;  // Allow Core 1 to resume
}

// Home both in sequence
void home_sensorless_XY_neg() {
  home_sensorless_X_neg();
  home_sensorless_Y_neg();
  Serial.println("HOMEXY COMPLETE");
}

// ================= VELOCITY CONTROL =====================

float rampVelocity(float current, float target, float dt) {
  float max_delta = MAX_ACCEL * dt;  // Max velocity change this frame
  float delta = target - current;

  if (delta > max_delta) {
    return current + max_delta;
  } else if (delta < -max_delta) {
    return current - max_delta;
  }
  return target;
}

// Update position based on velocity with acceleration limiting
// Called by Core 1 only
void updateVelocityControl() {
  if (!motors_enabled || !drivers_ok) return;
  if (is_homing) return;  // Core 0 is homing, don't interfere

  unsigned long now = millis();

  // Check if we should skip the timing gate (new command just arrived)
  bool force_update = force_velocity_update;
  if (force_update) {
    force_velocity_update = false;  // Clear the flag
  }

  // Only gate on timing if not forced by a new command
  if (!force_update && (now - last_move_time < VELOCITY_UPDATE_MS)) {
    return;
  }

  float dt = (now - last_move_time) / 1000.0f;  // seconds
  if (dt <= 0.0f) dt = 0.01f;  // Minimum dt to avoid division issues
  last_move_time = now;

  // Clamp commanded targets
  if (target_x_vel >  MAX_VELOCITY) target_x_vel =  MAX_VELOCITY;
  if (target_x_vel < -MAX_VELOCITY) target_x_vel = -MAX_VELOCITY;
  if (target_y_vel >  MAX_VELOCITY) target_y_vel =  MAX_VELOCITY;
  if (target_y_vel < -MAX_VELOCITY) target_y_vel = -MAX_VELOCITY;

  // Ramping
  current_x_vel = rampVelocity(current_x_vel, target_x_vel, dt);
  current_y_vel = rampVelocity(current_y_vel, target_y_vel, dt);

  // Compute position increment from velocity
  float dx = current_x_vel * dt;   // mm
  float dy = current_y_vel * dt;   // mm

  // === APPLY BOUNDING BOX (only for axes that have been homed) ===
  if (homed_x) {
    float x_new = cur_x_mm + dx;
    if (x_new < X_MIN) {
      dx = X_MIN - cur_x_mm;
      current_x_vel = 0.0f;
      if (target_x_vel < 0.0f) target_x_vel = 0.0f;
    } else if (x_new > X_MAX) {
      dx = X_MAX - cur_x_mm;
      current_x_vel = 0.0f;
      if (target_x_vel > 0.0f) target_x_vel = 0.0f;
    }
  }

  if (homed_y) {
    float y_new = cur_y_mm + dy;
    if (y_new < Y_MIN) {
      dy = Y_MIN - cur_y_mm;
      current_y_vel = 0.0f;
      if (target_y_vel < 0.0f) target_y_vel = 0.0f;
    } else if (y_new > Y_MAX) {
      dy = Y_MAX - cur_y_mm;
      current_y_vel = 0.0f;
      if (target_y_vel > 0.0f) target_y_vel = 0.0f;
    }
  }

  if (dx == 0.0f && dy == 0.0f) return;

  // Convert to CoreXY A/B steps
  float dA_f = (dx + dy) * STEPS_PER_MM;  // A = X + Y
  float dB_f = (dx - dy) * STEPS_PER_MM;  // B = X - Y

  int32_t dA = (int32_t)lroundf(dA_f);
  int32_t dB = (int32_t)lroundf(dB_f);

  if (dA == 0 && dB == 0) return;

  // Approximate step delay based on speed
  float speed = sqrtf(current_x_vel * current_x_vel + current_y_vel * current_y_vel);
  uint16_t step_delay = (speed > 50.0f) ? 50 : 150;  // tweak as needed

  int32_t maxSteps = max(abs(dA), abs(dB));
  int32_t stepsExecuted = step_corexy_AB_delta(dA, dB, step_delay);

  // Update estimated position based on ACTUAL steps executed, not planned
  // This prevents position drift when steps are capped or aborted
  if (maxSteps > 0 && stepsExecuted > 0) {
    float fraction = (float)stepsExecuted / (float)maxSteps;
    cur_x_mm += dx * fraction;
    cur_y_mm += dy * fraction;
  }
}

// ============= CORE 0: SERIAL HANDLING ===================
//
// Protocol:
//   "vx vy\n"        -> velocity command (mm/s, floats, space or comma separated)
//   "HOMEX\n"        -> sensorless home X- (backoff, then center X)
//   "HOMEY\n"        -> sensorless home Y- (backoff, then center Y)
//   "HOME\n"/"HOMEXY\n" -> home X then Y
//
// Core 0 handles all serial communication so it's always responsive,
// even when Core 1 is busy stepping motors.

void processSerialCore0() {
  static char buf[64];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') {
      continue;  // ignore CR
    }

    if (c == '\n') {
      if (idx == 0) {
        // empty line
        continue;
      }

      buf[idx] = '\0';
      idx = 0;

      float vx = 0.0f, vy = 0.0f;
      int parsed = sscanf(buf, "%f %f", &vx, &vy);
      if (parsed != 2) {
        parsed = sscanf(buf, "%f,%f", &vx, &vy);
      }

      if (parsed == 2) {
        // Signal Core 1 to abort current motion immediately
        abort_current_motion = true;
        force_velocity_update = true;  // Bypass timing gate for immediate response

        // Update shared velocity (thread-safe)
        mutex_enter_blocking(&velocity_mutex);
        shared_target_x_vel = vx;
        shared_target_y_vel = vy;
        new_velocity_available = true;
        mutex_exit(&velocity_mutex);

        motors_enabled = true;
        last_cmd_time = millis();
      } else {
        // String command - homing runs on Core 0 (blocking is OK during homing)
        if (strcmp(buf, "HOMEX") == 0) {
          home_sensorless_X_neg();
        } else if (strcmp(buf, "HOMEY") == 0) {
          home_sensorless_Y_neg();
        } else if (strcmp(buf, "HOME") == 0 || strcmp(buf, "HOMEXY") == 0) {
          home_sensorless_XY_neg();
        }
      }

    } else {
      if (idx < sizeof(buf) - 1) {
        buf[idx++] = c;
      }
      // else: overflow -> silently ignore extra chars
    }
  }
}

// Check for serial timeout - stop if no bytes at all for a while
void checkSerialTimeout() {
  static unsigned long last_serial_time = 0;
  const unsigned long SERIAL_TIMEOUT_MS = 2000;  // 2 seconds

  if (Serial.available()) {
    last_serial_time = millis();
  }

  if (motors_enabled && (millis() - last_serial_time > SERIAL_TIMEOUT_MS)) {
    // Signal Core 1 to stop
    mutex_enter_blocking(&velocity_mutex);
    shared_target_x_vel = 0.0f;
    shared_target_y_vel = 0.0f;
    new_velocity_available = true;
    mutex_exit(&velocity_mutex);
  }
}

// ======================= CORE 0 SETUP/LOOP =====================
// Core 0: Handles serial communication - always responsive

void setup() {
  // Initialize mutex for inter-core communication
  mutex_init(&velocity_mutex);

  Serial.begin(115200);
  while (!Serial) {}

  pinMode(A_STEP_PIN, OUTPUT);
  pinMode(A_DIR_PIN,  OUTPUT);
  pinMode(A_EN_PIN,   OUTPUT);
  pinMode(B_STEP_PIN, OUTPUT);
  pinMode(B_DIR_PIN,  OUTPUT);
  pinMode(B_EN_PIN,   OUTPUT);

  // Enable drivers (active low EN)
  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  motors_enabled = true;

  pinMode(A_DIAG_PIN, INPUT_PULLUP);
  pinMode(B_DIAG_PIN, INPUT_PULLUP);

  Serial1.setRX(TMC_UART_RX);
  Serial1.setTX(TMC_UART_TX);
  Serial1.begin(115200);
  delay(100);

  config_tmc_driver(drvB);
  config_tmc_driver(drvA);
  check_tmc_connections();

  last_cmd_time = millis();

  Serial.println("READY VELOCITY + HOMEX/HOMEY + CENTERED (DUAL CORE)");
}

void loop() {
  // Core 0: Only handles serial - never blocked by motor stepping
  processSerialCore0();
  checkSerialTimeout();
}

// ======================= CORE 1 SETUP/LOOP =====================
// Core 1: Handles motor stepping - can block without affecting serial

void setup1() {
  // Wait for Core 0 to finish initialization
  delay(500);
  last_move_time = millis();
}

void loop1() {
  // Check for new velocity command from Core 0
  mutex_enter_blocking(&velocity_mutex);
  if (new_velocity_available) {
    target_x_vel = shared_target_x_vel;
    target_y_vel = shared_target_y_vel;
    new_velocity_available = false;
    abort_current_motion = false;  // Clear abort flag, we have the new command
  }
  mutex_exit(&velocity_mutex);

  // Run the (potentially blocking) velocity control
  // If a new command arrives, step_corexy_AB_delta will exit early
  updateVelocityControl();
}
