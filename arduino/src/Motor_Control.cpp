#include <Arduino.h>
#include <TMCStepper.h>
#include <ArduinoJson.h>
#include <math.h>

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

// CoreXY direction config
#define X_NEG_A_DIR    HIGH
#define X_NEG_B_DIR    HIGH
#define Y_NEG_A_DIR    HIGH
#define Y_NEG_B_DIR    LOW

// Homing parameters
const uint8_t  HOME_SGTHRS        = 5;
const uint16_t HOME_STEP_DELAY_US = 1200;
const uint16_t BACKOFF_STEPS      = 800;
const uint16_t BACKOFF_DELAY_US   = 1000;
const uint32_t MAX_HOME_STEPS     = 44500;
const uint8_t  STALL_STOP_STEPS   = 15;

// Motion parameters
const float STEPS_PER_MM = 160.0f;   // Adjust for your setup
const float MAX_VELOCITY = 400.0f;   // mm/s max speed
const float MAX_ACCEL = 6000.0f;     // mm/s^2 max acceleration

// PID gains for position control
const float KP_POS = 15.0f;          // Proportional gain (higher = faster response)
const float KD_POS = 0.5f;           // Derivative gain (damping)
const float MIN_VELOCITY = 5.0f;     // Minimum movement speed (mm/s)

// CoreXY motor directions
const uint8_t A_DIR_POS = HIGH;
const uint8_t B_DIR_POS = HIGH;

// =========================================================

TMC2209Stepper drvA(&Serial1, R_SENSE, 0b00);
TMC2209Stepper drvB(&Serial1, R_SENSE, 0b01);

bool drivers_ok = false;
bool motors_enabled = false;
bool homed = false;

// Current position in mm
float cur_x_mm = 0.0f;
float cur_y_mm = 0.0f;

// Workspace limits (set by homing)
float x_min = 0.0f;
float x_max = 70.0f;  // Default, will be measured
float y_min = 0.0f;
float y_max = 35.0f;  // Default, will be measured

// Velocity control
float target_x_vel = 0.0f;  // mm/s - commanded velocity
float target_y_vel = 0.0f;  // mm/s
float current_x_vel = 0.0f; // mm/s - actual velocity (ramped)
float current_y_vel = 0.0f; // mm/s
unsigned long last_move_time = 0;
const uint16_t VELOCITY_UPDATE_MS = 10;  // Update position every 10ms (100Hz)

// Position target mode (for PID control)
bool position_mode = false;
float target_x_pos = 0.0f;  // Target position in mm
float target_y_pos = 0.0f;
float last_error_x = 0.0f;  // For derivative term
float last_error_y = 0.0f;

// Debug streaming
bool stream_debug = false;
unsigned long last_stream_time = 0;
const uint16_t STREAM_INTERVAL_MS = 80;  // Stream every 80ms when enabled

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
  drv.SGTHRS(HOME_SGTHRS);
  drv.shaft(false);
}

void check_tmc_connections() {
  uint8_t connA = drvA.test_connection();
  uint8_t connB = drvB.test_connection();
  drivers_ok = (connA == 0 && connB == 0);
}

// ================= MOTION HELPERS =======================

inline bool stall_A() { return digitalRead(A_DIAG_PIN) == HIGH; }
inline bool stall_B() { return digitalRead(B_DIAG_PIN) == HIGH; }

inline void step_both(uint16_t pulse_us) {
  digitalWrite(A_STEP_PIN, HIGH);
  digitalWrite(B_STEP_PIN, HIGH);
  delayMicroseconds(pulse_us);
  digitalWrite(A_STEP_PIN, LOW);
  digitalWrite(B_STEP_PIN, LOW);
}

// Step motors with CoreXY kinematics: A = X + Y, B = X - Y
void step_corexy_AB_delta(int32_t dA, int32_t dB, uint16_t step_delay_us) {
  int32_t stepsA = abs(dA);
  int32_t stepsB = abs(dB);
  int32_t maxSteps = max(stepsA, stepsB);

  if (maxSteps == 0) return;

  uint8_t a_dir = (dA >= 0) ? A_DIR_POS : !A_DIR_POS;
  uint8_t b_dir = (dB >= 0) ? B_DIR_POS : !B_DIR_POS;

  digitalWrite(A_DIR_PIN, a_dir);
  digitalWrite(B_DIR_PIN, b_dir);
  delayMicroseconds(20);

  float incA = (stepsA > 0) ? (float)stepsA / maxSteps : 0.0f;
  float incB = (stepsB > 0) ? (float)stepsB / maxSteps : 0.0f;
  float accA = 0.0f;
  float accB = 0.0f;

  for (int32_t i = 0; i < maxSteps; i++) {
    accA += incA;
    accB += incB;

    if (accA >= 1.0f) {
      digitalWrite(A_STEP_PIN, HIGH);
      delayMicroseconds(3);
      digitalWrite(A_STEP_PIN, LOW);
      accA -= 1.0f;
    }

    if (accB >= 1.0f) {
      digitalWrite(B_STEP_PIN, HIGH);
      delayMicroseconds(3);
      digitalWrite(B_STEP_PIN, LOW);
      accB -= 1.0f;
    }

    delayMicroseconds(step_delay_us);
  }
}

// Move to absolute XY position (boundary checked)
bool move_to_xy_mm(float x_target, float y_target, uint16_t step_delay_us = 500) {
  // Boundary check
  if (homed) {
    x_target = constrain(x_target, x_min, x_max);
    y_target = constrain(y_target, y_min, y_max);
  }

  float dx = x_target - cur_x_mm;
  float dy = y_target - cur_y_mm;

  // CoreXY kinematics: A = X + Y, B = X - Y
  float dA_f = (dx + dy) * STEPS_PER_MM;
  float dB_f = (dx - dy) * STEPS_PER_MM;

  int32_t dA = (int32_t)lroundf(dA_f);
  int32_t dB = (int32_t)lroundf(dB_f);

  if (dA == 0 && dB == 0) return true;

  step_corexy_AB_delta(dA, dB, step_delay_us);

  cur_x_mm = x_target;
  cur_y_mm = y_target;
  return true;
}

// ================= SENSORLESS HOMING ===================

void home_x_min_sensorless() {
  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(200);

  digitalWrite(A_DIR_PIN, X_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, X_NEG_B_DIR);
  delay(100);

  uint32_t steps = 0;
  uint8_t stall_count = 0;

  while (steps < MAX_HOME_STEPS) {
    step_both(5);
    delayMicroseconds(HOME_STEP_DELAY_US);

    if (stall_A() || stall_B()) {
      stall_count++;
      if (stall_count >= STALL_STOP_STEPS) break;
    } else {
      stall_count = 0;
    }
    steps++;
  }

  digitalWrite(A_EN_PIN, HIGH);
  digitalWrite(B_EN_PIN, HIGH);
  delay(100);

  // Back off
  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(50);
  digitalWrite(A_DIR_PIN, !X_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, !X_NEG_B_DIR);
  delay(100);

  for (uint16_t i = 0; i < BACKOFF_STEPS; i++) {
    step_both(5);
    delayMicroseconds(BACKOFF_DELAY_US);
  }

  cur_x_mm = 0.0f;
  x_min = 0.0f;
}

void home_x_max_sensorless() {
  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(200);

  digitalWrite(A_DIR_PIN, !X_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, !X_NEG_B_DIR);
  delay(100);

  uint32_t steps = 0;
  uint8_t stall_count = 0;
  float start_x = cur_x_mm;

  while (steps < MAX_HOME_STEPS) {
    step_both(5);
    delayMicroseconds(HOME_STEP_DELAY_US);

    if (stall_A() || stall_B()) {
      stall_count++;
      if (stall_count >= STALL_STOP_STEPS) break;
    } else {
      stall_count = 0;
    }
    steps++;
  }

  digitalWrite(A_EN_PIN, HIGH);
  digitalWrite(B_EN_PIN, HIGH);
  delay(100);

  // Calculate X max from steps traveled
  x_max = start_x + (steps / STEPS_PER_MM);

  // Back off
  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(50);
  digitalWrite(A_DIR_PIN, X_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, X_NEG_B_DIR);
  delay(100);

  for (uint16_t i = 0; i < BACKOFF_STEPS; i++) {
    step_both(5);
    delayMicroseconds(BACKOFF_DELAY_US);
  }
}

void home_y_min_sensorless() {
  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(200);

  digitalWrite(A_DIR_PIN, Y_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, Y_NEG_B_DIR);
  delay(100);

  uint32_t steps = 0;
  uint8_t stall_count = 0;

  while (steps < MAX_HOME_STEPS) {
    step_both(5);
    delayMicroseconds(HOME_STEP_DELAY_US);

    if (stall_A() || stall_B()) {
      stall_count++;
      if (stall_count >= STALL_STOP_STEPS) break;
    } else {
      stall_count = 0;
    }
    steps++;
  }

  digitalWrite(A_EN_PIN, HIGH);
  digitalWrite(B_EN_PIN, HIGH);
  delay(100);

  // Back off
  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(50);
  digitalWrite(A_DIR_PIN, !Y_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, !Y_NEG_B_DIR);
  delay(100);

  for (uint16_t i = 0; i < BACKOFF_STEPS; i++) {
    step_both(5);
    delayMicroseconds(BACKOFF_DELAY_US);
  }

  cur_y_mm = 0.0f;
  y_min = 0.0f;
}

void home_y_max_sensorless() {
  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(200);

  digitalWrite(A_DIR_PIN, !Y_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, !Y_NEG_B_DIR);
  delay(100);

  uint32_t steps = 0;
  uint8_t stall_count = 0;
  float start_y = cur_y_mm;

  while (steps < MAX_HOME_STEPS) {
    step_both(5);
    delayMicroseconds(HOME_STEP_DELAY_US);

    if (stall_A() || stall_B()) {
      stall_count++;
      if (stall_count >= STALL_STOP_STEPS) break;
    } else {
      stall_count = 0;
    }
    steps++;
  }

  digitalWrite(A_EN_PIN, HIGH);
  digitalWrite(B_EN_PIN, HIGH);
  delay(100);

  // Calculate Y max from steps traveled
  y_max = start_y + (steps / STEPS_PER_MM);

  // Back off
  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(50);
  digitalWrite(A_DIR_PIN, Y_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, Y_NEG_B_DIR);
  delay(100);

  for (uint16_t i = 0; i < BACKOFF_STEPS; i++) {
    step_both(5);
    delayMicroseconds(BACKOFF_DELAY_US);
  }
}

void calibrate_workspace() {
  home_x_min_sensorless();
  delay(300);
  home_y_min_sensorless();
  delay(300);
  home_x_max_sensorless();
  delay(300);
  home_y_max_sensorless();
  delay(300);
  home_x_min_sensorless();
  delay(300);
  home_y_min_sensorless();
  delay(300);
  
  homed = true;
}

// ================= JSON COMMAND HANDLING ================

void sendResponse(const char* cmd, const char* status, const char* message = nullptr) {
  JsonDocument doc;
  doc["cmd"] = cmd;
  doc["status"] = status;
  if (message) doc["message"] = message;
  doc["x"] = cur_x_mm;
  doc["y"] = cur_y_mm;
  doc["homed"] = homed;
  doc["enabled"] = motors_enabled;
  serializeJson(doc, Serial);
  Serial.println();
}

void handlePingCommand() {
  sendResponse("ping", "ok", "pong");
}

void handleEnableCommand(JsonDocument& doc) {
  bool state = doc["state"] | true;
  
  digitalWrite(A_EN_PIN, state ? LOW : HIGH);
  digitalWrite(B_EN_PIN, state ? LOW : HIGH);
  motors_enabled = state;
  
  sendResponse("enable", "ok", state ? "Motors enabled" : "Motors disabled");
}

void handleHomeCommand(JsonDocument& doc) {
  if (!motors_enabled) {
    sendResponse("home", "error", "Motors not enabled");
    return;
  }
  
  // Send starting message
  JsonDocument startDoc;
  startDoc["cmd"] = "home";
  startDoc["status"] = "in_progress";
  startDoc["message"] = "Starting homing sequence";
  serializeJson(startDoc, Serial);
  Serial.println();
  
  calibrate_workspace();
  
  // Send completion message
  sendResponse("home", "ok", "Homing complete");
}

void handleMoveCommand(JsonDocument& doc) {
  if (!motors_enabled) {
    sendResponse("move", "error", "Motors not enabled");
    return;
  }
  
  float x = doc["x"] | cur_x_mm;
  float y = doc["y"] | cur_y_mm;
  
  move_to_xy_mm(x, y, 300);
  sendResponse("move", "ok");
}

void handleVelocityCommand(JsonDocument& doc) {
  if (!motors_enabled) {
    sendResponse("velocity", "error", "Motors not enabled");
    return;
  }
  
  // Disable position mode when receiving velocity commands
  position_mode = false;
  
  target_x_vel = doc["x_vel"] | 0.0f;
  target_y_vel = doc["y_vel"] | 0.0f;
  
  // Constrain velocity to max limits
  target_x_vel = constrain(target_x_vel, -MAX_VELOCITY, MAX_VELOCITY);
  target_y_vel = constrain(target_y_vel, -MAX_VELOCITY, MAX_VELOCITY);
  
  sendResponse("velocity", "ok");
}

void handleStopCommand() {
  position_mode = false;
  target_x_vel = 0.0f;
  target_y_vel = 0.0f;
  current_x_vel = 0.0f;
  current_y_vel = 0.0f;
  sendResponse("stop", "ok");
}

void handlePositionCommand(JsonDocument& doc) {
  if (!motors_enabled) {
    sendResponse("position", "error", "Motors not enabled");
    return;
  }
  
  // Set target position for PID control
  target_x_pos = doc["x"] | cur_x_mm;
  target_y_pos = doc["y"] | cur_y_mm;
  
  // Clamp to workspace
  if (homed) {
    target_x_pos = constrain(target_x_pos, x_min, x_max);
    target_y_pos = constrain(target_y_pos, y_min, y_max);
  }
  
  // Enable position mode
  position_mode = true;
  last_error_x = 0.0f;
  last_error_y = 0.0f;
  
  sendResponse("position", "ok");
}

void handleLocalizeCommand(JsonDocument &doc) {
  // Set position from vision-based localization
  // This allows recovery from arbitrary positions without full homing
  if (!doc["x"].is<float>() || !doc["y"].is<float>()) {
    sendResponse("localize", "failed", "Missing x or y");
    return;
  }
  
  float new_x = doc["x"] | 0.0f;
  float new_y = doc["y"] | 0.0f;
  
  // Validate coordinates are within known bounds
  if (new_x < x_min || new_x > x_max || new_y < y_min || new_y > y_max) {
    sendResponse("localize", "failed", "Position out of bounds");
    return;
  }
  
  // Update current position
  cur_x_mm = new_x;
  cur_y_mm = new_y;
  
  // Stop any ongoing motion
  target_x_vel = 0.0f;
  target_y_vel = 0.0f;
  
  sendResponse("localize", "ok");
}

void handleSetWorkspaceCommand(JsonDocument &doc) {
  // Manually set workspace limits (use when homing gives wrong values)
  float new_x_max = doc["x_max"] | x_max;
  float new_y_max = doc["y_max"] | y_max;
  float new_x_min = doc["x_min"] | x_min;
  float new_y_min = doc["y_min"] | y_min;
  
  x_min = new_x_min;
  x_max = new_x_max;
  y_min = new_y_min;
  y_max = new_y_max;
  
  // CRITICAL: Reset current position to center of new workspace
  // This prevents the robot from thinking it's outside boundaries
  cur_x_mm = (new_x_min + new_x_max) / 2.0f;
  cur_y_mm = (new_y_min + new_y_max) / 2.0f;
  
  // Stop all motion
  target_x_vel = 0.0f;
  target_y_vel = 0.0f;
  current_x_vel = 0.0f;
  current_y_vel = 0.0f;
  
  // Mark as homed since we now have valid limits
  homed = true;
  
  sendResponse("set_workspace", "ok");
}

void handleStatusCommand() {
  JsonDocument doc;
  doc["cmd"] = "status";
  doc["status"] = "ok";
  doc["x"] = cur_x_mm;
  doc["y"] = cur_y_mm;
  doc["x_min"] = x_min;
  doc["x_max"] = x_max;
  doc["y_min"] = y_min;
  doc["y_max"] = y_max;
  doc["homed"] = homed;
  doc["enabled"] = motors_enabled;
  doc["x_vel"] = current_x_vel;
  doc["y_vel"] = current_y_vel;
  doc["pos_mode"] = position_mode;
  doc["streaming"] = stream_debug;
  serializeJson(doc, Serial);
  Serial.println();
}

void handleStreamCommand(JsonDocument& doc) {
  stream_debug = doc["state"] | false;
  sendResponse("stream", "ok", stream_debug ? "Debug streaming ON" : "Debug streaming OFF");
}

void streamMotionDebug() {
  // Only stream if enabled and enough time has passed
  if (!stream_debug) return;
  
  unsigned long now = millis();
  if (now - last_stream_time < STREAM_INTERVAL_MS) return;
  last_stream_time = now;
  
  // Only stream if there's actual motion (not when stopped)
  float speed = sqrtf(current_x_vel * current_x_vel + current_y_vel * current_y_vel);
  if (speed < 3.0f && !position_mode) return;  // Don't spam when stopped
  
  JsonDocument doc;
  doc["debug"] = "motion";
  doc["x"] = cur_x_mm;
  doc["y"] = cur_y_mm;
  doc["vx"] = current_x_vel;
  doc["vy"] = current_y_vel;
  
  // Direction indicators
  String dir = "";
  if (current_y_vel > 5.0f) dir += "UP ";
  if (current_y_vel < -5.0f) dir += "DOWN ";
  if (current_x_vel > 5.0f) dir += "RIGHT ";
  if (current_x_vel < -5.0f) dir += "LEFT ";
  if (dir == "") dir = "IDLE";
  doc["dir"] = dir;
  doc["speed"] = speed;
  
  // Include workspace limits for debugging
  doc["x_max"] = x_max;
  doc["y_max"] = y_max;
  
  serializeJson(doc, Serial);
  Serial.println();
}

void processSerialCommands() {
  static String jsonBuffer = "";
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n') {
      if (jsonBuffer.length() > 0) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, jsonBuffer);
        
        if (error) {
          sendResponse("error", "failed", "Invalid JSON");
        } else {
          const char* cmd = doc["cmd"];
          
          if (!cmd) {
            sendResponse("error", "failed", "Missing cmd field");
          } else if (strcmp(cmd, "ping") == 0) {
            handlePingCommand();
          } else if (strcmp(cmd, "enable") == 0) {
            handleEnableCommand(doc);
          } else if (strcmp(cmd, "home") == 0) {
            handleHomeCommand(doc);
          } else if (strcmp(cmd, "move") == 0) {
            handleMoveCommand(doc);
          } else if (strcmp(cmd, "velocity") == 0) {
            handleVelocityCommand(doc);
          } else if (strcmp(cmd, "stop") == 0) {
            handleStopCommand();
          } else if (strcmp(cmd, "localize") == 0) {
            handleLocalizeCommand(doc);
          } else if (strcmp(cmd, "status") == 0) {
            handleStatusCommand();
          } else if (strcmp(cmd, "position") == 0) {
            handlePositionCommand(doc);
          } else if (strcmp(cmd, "stream") == 0) {
            handleStreamCommand(doc);
          } else if (strcmp(cmd, "set_workspace") == 0) {
            handleSetWorkspaceCommand(doc);
          } else {
            sendResponse("error", "failed", "Unknown command");
          }
        }
        
        jsonBuffer = "";
      }
    } else {
      jsonBuffer += c;
      if (jsonBuffer.length() > 512) {
        jsonBuffer = "";
      }
    }
  }
}

// Ramp velocity with acceleration limit
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
void updateVelocityControl() {
  unsigned long now = millis();
  
  if (now - last_move_time >= VELOCITY_UPDATE_MS) {
    float dt = (now - last_move_time) / 1000.0f;  // seconds
    
    // === POSITION MODE: PID control to target position ===
    if (position_mode) {
      // Calculate position error
      float error_x = target_x_pos - cur_x_mm;
      float error_y = target_y_pos - cur_y_mm;
      
      // Derivative of error
      float d_error_x = (error_x - last_error_x) / dt;
      float d_error_y = (error_y - last_error_y) / dt;
      last_error_x = error_x;
      last_error_y = error_y;
      
      // PD control: velocity = Kp * error + Kd * d_error
      // Farther away = faster, closer = slower
      target_x_vel = (KP_POS * error_x) + (KD_POS * d_error_x);
      target_y_vel = (KP_POS * error_y) + (KD_POS * d_error_y);
      
      // Clamp to max velocity
      float speed = sqrtf(target_x_vel * target_x_vel + target_y_vel * target_y_vel);
      if (speed > MAX_VELOCITY) {
        float scale = MAX_VELOCITY / speed;
        target_x_vel *= scale;
        target_y_vel *= scale;
      }
      
      // Stop if close enough to target
      float dist = sqrtf(error_x * error_x + error_y * error_y);
      if (dist < 0.5f) {  // Within 0.5mm
        target_x_vel = 0.0f;
        target_y_vel = 0.0f;
        current_x_vel = 0.0f;
        current_y_vel = 0.0f;
      }
    }
    
    // === VELOCITY RAMPING: Apply acceleration limits ===
    current_x_vel = rampVelocity(current_x_vel, target_x_vel, dt);
    current_y_vel = rampVelocity(current_y_vel, target_y_vel, dt);
    
    // === MOTION: Apply velocity to position ===
    if (current_x_vel != 0.0f || current_y_vel != 0.0f) {
      float new_x = cur_x_mm + (current_x_vel * dt);
      float new_y = cur_y_mm + (current_y_vel * dt);
      
      // Use HOMING-derived limits as ground truth (with safety margin)
      float margin = 10.0f;  // Pre-emptive stop margin in mm
      
      // Boundary check - stop BEFORE hitting edges
      bool at_x_min = (new_x <= x_min + margin);
      bool at_x_max = (new_x >= x_max - margin);
      bool at_y_min = (new_y <= y_min + margin);
      bool at_y_max = (new_y >= y_max - margin);
      
      // Hard clamp to homing limits
      new_x = constrain(new_x, x_min, x_max);
      new_y = constrain(new_y, y_min, y_max);
      
      // Stop velocity if within margin of boundary
      if (at_x_min && current_x_vel < 0) { current_x_vel = 0.0f; target_x_vel = 0.0f; }
      if (at_x_max && current_x_vel > 0) { current_x_vel = 0.0f; target_x_vel = 0.0f; }
      if (at_y_min && current_y_vel < 0) { current_y_vel = 0.0f; target_y_vel = 0.0f; }
      if (at_y_max && current_y_vel > 0) { current_y_vel = 0.0f; target_y_vel = 0.0f; }
      
      // Calculate step delay based on velocity (faster = shorter delay)
      float speed = sqrtf(current_x_vel * current_x_vel + current_y_vel * current_y_vel);
      uint16_t step_delay = (speed > 50.0f) ? 50 : 150;  // Faster stepping at high speed
      
      move_to_xy_mm(new_x, new_y, step_delay);
    }
    
    last_move_time = now;
  }
}

// Check for serial timeout - stop if no commands received
void checkSerialTimeout() {
  static unsigned long last_serial_time = 0;
  const unsigned long SERIAL_TIMEOUT_MS = 2000;  // 2 seconds
  
  if (Serial.available()) {
    last_serial_time = millis();
  }
  
  // If no serial data for 2 seconds, stop motors
  if (motors_enabled && (millis() - last_serial_time > SERIAL_TIMEOUT_MS)) {
    target_x_vel = 0.0f;
    target_y_vel = 0.0f;
  }
}

// ===================== SETUP/LOOP ======================

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(A_STEP_PIN, OUTPUT);
  pinMode(A_DIR_PIN,  OUTPUT);
  pinMode(A_EN_PIN,   OUTPUT);
  pinMode(B_STEP_PIN, OUTPUT);
  pinMode(B_DIR_PIN,  OUTPUT);
  pinMode(B_EN_PIN,   OUTPUT);

  digitalWrite(A_EN_PIN, HIGH);
  digitalWrite(B_EN_PIN, HIGH);

  pinMode(A_DIAG_PIN, INPUT_PULLUP);
  pinMode(B_DIAG_PIN, INPUT_PULLUP);

  Serial1.setRX(TMC_UART_RX);
  Serial1.setTX(TMC_UART_TX);
  Serial1.begin(115200);
  delay(100);

  config_tmc_driver(drvB);
  config_tmc_driver(drvA);
  check_tmc_connections();

  JsonDocument doc;
  doc["status"] = "ready";
  doc["drivers_ok"] = drivers_ok;
  serializeJson(doc, Serial);
  Serial.println();
}

void loop() {
  processSerialCommands();
  updateVelocityControl();
  streamMotionDebug();   // Stream debug info if enabled
  checkSerialTimeout();  // Stop motors if Python disconnects
}
