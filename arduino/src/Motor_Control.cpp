#include <Arduino.h>
#include <TMCStepper.h>
#include <math.h>

// ====================== USER CONFIG ======================

// TMC sense resistor (BTT 2209 is usually 0.11Ω)
#define R_SENSE        0.11f

// UART bus (XIAO RP2350)
// MCU_TX (D6) -> 2.2k -> PDN/UART bus on both TMC2209s
// MCU_RX (D7) -> PDN/UART bus
#define TMC_UART_RX    D7     // PDN bus -> XIAO RX
#define TMC_UART_TX    D6     // XIAO TX -> 2.2k -> PDN bus

// CoreXY motor A
#define A_STEP_PIN     D1
#define A_DIR_PIN      D0
#define A_EN_PIN       D4

// CoreXY motor B
#define B_STEP_PIN     D5
#define B_DIR_PIN      D2
#define B_EN_PIN       D8

// DIAG pins for sensorless homing (one per driver)
#define A_DIAG_PIN     D10
#define B_DIAG_PIN     D9

// ---------- CoreXY direction config ----------
// Tune these so:
//  - X_NEG_* really moves toward X-min
//  - Y_NEG_* really moves toward Y-min

// "X- move" direction (CoreXY: both motors same way)
#define X_NEG_A_DIR    HIGH
#define X_NEG_B_DIR    HIGH

// "Y- move" direction (CoreXY: motors opposite ways)
#define Y_NEG_A_DIR    HIGH
#define Y_NEG_B_DIR    LOW

// Homing parameters - Tuned for actual edge detection
const uint8_t  HOME_SGTHRS        = 5;      // LOWER = more sensitive (0-255, typically 0-10 for homing)
const uint16_t HOME_STEP_DELAY_US = 1200;   // Slower = more reliable stall detection
const uint16_t BACKOFF_STEPS      = 800;    // Steps to back off after stall
const uint16_t BACKOFF_DELAY_US   = 1000;   // Backoff speed
const uint32_t MAX_HOME_STEPS     = 45000;  // Increased safety limit to reach all edges
const uint8_t  STALL_STOP_STEPS   = 15;     // Require sustained stall (filter noise)

// --------- Motion / circle parameters ----------

// Approx steps/mm (adjust for your pulley & microstepping)
const float STEPS_PER_MM = 160.0f;   // 0.9° motor, 16x, 20T pulley -> ~160

// Circle settings (in mm, in XY space)
// After homing, origin (0,0) is at bottom-left corner
const float CIRCLE_RADIUS_MM    = 10.0f;  // Small radius for safety
const float CIRCLE_CENTER_X_MM  = 20.0f;  // Safe distance from left edge
const float CIRCLE_CENTER_Y_MM  = 12.0f;  // Safe distance from bottom edge

const int   CIRCLE_SEGMENTS        = 240;   // resolution of the circle
const uint16_t CIRCLE_STEP_DELAY_US = 250;  // Faster speed (was 500)

// =========================================================

// On RP2350/RP2040 we use Serial1 directly for the TMC bus

TMC2209Stepper drvA(&Serial1, R_SENSE, 0b00);  // A: MS1=0, MS2=0
TMC2209Stepper drvB(&Serial1, R_SENSE, 0b01);  // B: MS1=1, MS2=0 (example)

// Global flag: only home if both drivers respond
bool drivers_ok = false;

// Track our "logical" XY position in mm
static float cur_x_mm = 0.0f;
static float cur_y_mm = 0.0f;

// For motor "positive" directions (belt direction)
// If circle runs mirrored, you can flip these.
const uint8_t A_DIR_POS = HIGH;
const uint8_t B_DIR_POS = HIGH;

// ------------------ TMC config helpers -------------------

void config_tmc_driver(TMC2209Stepper &drv) {
  drv.begin();
  drv.pdn_disable(true);
  drv.I_scale_analog(false);

  drv.toff(4);
  drv.blank_time(24);
  drv.microsteps(16);
  drv.rms_current(1500);      // Increase if motors are weak

  drv.en_spreadCycle(false);
  drv.pwm_autoscale(true);

  // StallGuard config
  drv.TCOOLTHRS(0xFFFFF);     // Enable StallGuard (DIAG output when velocity > TCOOLTHRS)
  drv.SGTHRS(HOME_SGTHRS);    // StallGuard threshold
  drv.shaft(false);           // Motor direction
}

// Simple UART connection check
void check_tmc_connections() {
  Serial.println(F("\n=== TMC2209 UART connection check ==="));

  // Driver A
  uint8_t connA = drvA.test_connection();
  Serial.print(F("Driver A (addr 0b00): "));
  if (connA == 0)      Serial.print(F("OK"));
  else if (connA == 1) Serial.print(F("No connection"));
  else if (connA == 2) Serial.print(F("Bad response"));
  else                 Serial.print(connA);
  Serial.print(F("  | IFCNT="));
  Serial.println(drvA.IFCNT());

  // Driver B
  uint8_t connB = drvB.test_connection();
  Serial.print(F("Driver B (addr 0b01): "));
  if (connB == 0)      Serial.print(F("OK"));
  else if (connB == 1) Serial.print(F("No connection"));
  else if (connB == 2) Serial.print(F("Bad response"));
  else                 Serial.print(connB);
  Serial.print(F("  | IFCNT="));
  Serial.println(drvB.IFCNT());

  if (connA == 0 && connB == 0) {
    drivers_ok = true;
    Serial.println(F("=> Both drivers respond over UART. Continuing.\n"));
  } else {
    drivers_ok = false;
    Serial.println(F("=> UART check FAILED. Skipping motion.\n"));
  }
}

// ------------------- CoreXY helpers ----------------------

// DIAG is treated as HIGH on stall / error.
inline bool stall_A() {
  return digitalRead(A_DIAG_PIN) == HIGH;
}

inline bool stall_B() {
  return digitalRead(B_DIAG_PIN) == HIGH;
}

// Step helper: pulse both motors once
inline void step_both(uint16_t pulse_us) {
  digitalWrite(A_STEP_PIN, HIGH);
  digitalWrite(B_STEP_PIN, HIGH);
  delayMicroseconds(pulse_us);
  digitalWrite(A_STEP_PIN, LOW);
  digitalWrite(B_STEP_PIN, LOW);
}

// Step helper for arbitrary delta in A/B (CoreXY space)
void step_corexy_AB_delta(int32_t dA, int32_t dB) {
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

    delayMicroseconds(CIRCLE_STEP_DELAY_US);
  }
}

// Move in XY (mm) using CoreXY mapping
// CoreXY: A = X + Y, B = X - Y (up to sign conventions)
void move_to_xy_mm(float x_target, float y_target) {
  float dx = x_target - cur_x_mm;
  float dy = y_target - cur_y_mm;

  float dA_f = (dx + dy) * STEPS_PER_MM;
  float dB_f = (dx - dy) * STEPS_PER_MM;

  int32_t dA = (int32_t)lroundf(dA_f);
  int32_t dB = (int32_t)lroundf(dB_f);

  if (dA == 0 && dB == 0) return;

  step_corexy_AB_delta(dA, dB);

  cur_x_mm = x_target;
  cur_y_mm = y_target;
}

// --------------- Axis homing functions -------------------

// Home to X-min - WITH DEBUG OUTPUT
void home_x_min_sensorless() {
  Serial.println(F("\n=== Homing X-min (CoreXY, sensorless) ==="));

  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(200);  // Longer enable delay

  digitalWrite(A_DIR_PIN, X_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, X_NEG_B_DIR);
  delay(100);  // Longer direction setup

  uint32_t steps = 0;
  uint8_t stall_count = 0;
  
  Serial.println(F("Moving toward X-min..."));

  while (steps < MAX_HOME_STEPS) {
    step_both(5);
    delayMicroseconds(HOME_STEP_DELAY_US);

    bool a_stall = stall_A();
    bool b_stall = stall_B();
    
    // Debug output every 1000 steps
    if (steps % 1000 == 0) {
      Serial.print(F("Steps: "));
      Serial.print(steps);
      Serial.print(F(" | DIAG_A: "));
      Serial.print(a_stall ? "HIGH" : "LOW");
      Serial.print(F(" | DIAG_B: "));
      Serial.println(b_stall ? "HIGH" : "LOW");
    }

    if (a_stall || b_stall) {
      stall_count++;
      if (stall_count >= STALL_STOP_STEPS) {
        Serial.print(F("X-min STALL detected at steps="));
        Serial.println(steps);
        break;
      }
    } else {
      stall_count = 0;
    }
    steps++;
  }

  if (stall_count < STALL_STOP_STEPS) {
    Serial.println(F("!! X-min homing aborted: MAX_HOME_STEPS reached"));
    Serial.println(F("!! Check: 1) Motors moving? 2) DIAG wiring? 3) SGTHRS too high?"));
    return;
  }

  // STOP motors immediately after stall
  digitalWrite(A_EN_PIN, HIGH);  // Disable motors
  digitalWrite(B_EN_PIN, HIGH);
  delay(100);

  // Back off
  Serial.println(F("Backing off from X-min endstop..."));
  digitalWrite(A_EN_PIN, LOW);   // Re-enable for backoff
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
  Serial.println(F("X-min homing complete. X=0 set here."));
}

// Home to X-max - WITH DEBUG OUTPUT
void home_x_max_sensorless() {
  Serial.println(F("\n=== Homing X-max (CoreXY, sensorless) ==="));

  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(200);

  digitalWrite(A_DIR_PIN, !X_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, !X_NEG_B_DIR);
  delay(100);

  uint32_t steps = 0;
  uint8_t stall_count = 0;
  
  Serial.println(F("Moving toward X-max..."));

  while (steps < MAX_HOME_STEPS) {
    step_both(5);
    delayMicroseconds(HOME_STEP_DELAY_US);

    bool a_stall = stall_A();
    bool b_stall = stall_B();
    
    if (steps % 1000 == 0) {
      Serial.print(F("Steps: "));
      Serial.print(steps);
      Serial.print(F(" | DIAG_A: "));
      Serial.print(a_stall ? "HIGH" : "LOW");
      Serial.print(F(" | DIAG_B: "));
      Serial.println(b_stall ? "HIGH" : "LOW");
    }

    if (a_stall || b_stall) {
      stall_count++;
      if (stall_count >= STALL_STOP_STEPS) {
        Serial.print(F("X-max STALL detected at steps="));
        Serial.println(steps);
        break;
      }
    } else {
      stall_count = 0;
    }
    steps++;
  }

  if (stall_count < STALL_STOP_STEPS) {
    Serial.println(F("!! X-max homing aborted: MAX_HOME_STEPS reached"));
    Serial.println(F("!! Check: 1) Motors moving? 2) DIAG wiring? 3) SGTHRS too high?"));
    return;
  }

  // STOP motors immediately after stall
  digitalWrite(A_EN_PIN, HIGH);
  digitalWrite(B_EN_PIN, HIGH);
  delay(100);

  Serial.println(F("Backing off from X-max endstop..."));
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

  Serial.println(F("X-max homing complete."));
}

// Home to Y-min - WITH DEBUG OUTPUT
void home_y_min_sensorless() {
  Serial.println(F("\n=== Homing Y-min (CoreXY, sensorless) ==="));

  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(200);

  digitalWrite(A_DIR_PIN, Y_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, Y_NEG_B_DIR);
  delay(100);

  uint32_t steps = 0;
  uint8_t stall_count = 0;
  
  Serial.println(F("Moving toward Y-min..."));

  while (steps < MAX_HOME_STEPS) {
    step_both(5);
    delayMicroseconds(HOME_STEP_DELAY_US);

    bool a_stall = stall_A();
    bool b_stall = stall_B();
    
    if (steps % 1000 == 0) {
      Serial.print(F("Steps: "));
      Serial.print(steps);
      Serial.print(F(" | DIAG_A: "));
      Serial.print(a_stall ? "HIGH" : "LOW");
      Serial.print(F(" | DIAG_B: "));
      Serial.println(b_stall ? "HIGH" : "LOW");
    }

    if (a_stall || b_stall) {
      stall_count++;
      if (stall_count >= STALL_STOP_STEPS) {
        Serial.print(F("Y-min STALL detected at steps="));
        Serial.println(steps);
        break;
      }
    } else {
      stall_count = 0;
    }
    steps++;
  }

  if (stall_count < STALL_STOP_STEPS) {
    Serial.println(F("!! Y-min homing aborted: MAX_HOME_STEPS reached"));
    Serial.println(F("!! Check: 1) Motors moving? 2) DIAG wiring? 3) SGTHRS too high?"));
    return;
  }

  // STOP motors immediately after stall
  digitalWrite(A_EN_PIN, HIGH);
  digitalWrite(B_EN_PIN, HIGH);
  delay(100);

  Serial.println(F("Backing off from Y-min endstop..."));
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
  Serial.println(F("Y-min homing complete. Y=0 set here."));
}

// Home to Y-max - WITH DEBUG OUTPUT
void home_y_max_sensorless() {
  Serial.println(F("\n=== Homing Y-max (CoreXY, sensorless) ==="));

  digitalWrite(A_EN_PIN, LOW);
  digitalWrite(B_EN_PIN, LOW);
  delay(200);

  digitalWrite(A_DIR_PIN, !Y_NEG_A_DIR);
  digitalWrite(B_DIR_PIN, !Y_NEG_B_DIR);
  delay(100);

  uint32_t steps = 0;
  uint8_t stall_count = 0;
  
  Serial.println(F("Moving toward Y-max..."));

  while (steps < MAX_HOME_STEPS) {
    step_both(5);
    delayMicroseconds(HOME_STEP_DELAY_US);

    bool a_stall = stall_A();
    bool b_stall = stall_B();
    
    if (steps % 1000 == 0) {
      Serial.print(F("Steps: "));
      Serial.print(steps);
      Serial.print(F(" | DIAG_A: "));
      Serial.print(a_stall ? "HIGH" : "LOW");
      Serial.print(F(" | DIAG_B: "));
      Serial.println(b_stall ? "HIGH" : "LOW");
    }

    if (a_stall || b_stall) {
      stall_count++;
      if (stall_count >= STALL_STOP_STEPS) {
        Serial.print(F("Y-max STALL detected at steps="));
        Serial.println(steps);
        break;
      }
    } else {
      stall_count = 0;
    }
    steps++;
  }

  if (stall_count < STALL_STOP_STEPS) {
    Serial.println(F("!! Y-max homing aborted: MAX_HOME_STEPS reached"));
    Serial.println(F("!! Check: 1) Motors moving? 2) DIAG wiring? 3) SGTHRS too high?"));
    return;
  }

  // STOP motors immediately after stall
  digitalWrite(A_EN_PIN, HIGH);
  digitalWrite(B_EN_PIN, HIGH);
  delay(100);

  Serial.println(F("Backing off from Y-max endstop..."));
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

  Serial.println(F("Y-max homing complete."));
}

// Fast calibration: Only map opposite corners (X-min/Y-min and X-max/Y-max)
void calibrate_workspace() {
  Serial.println(F("\n========================================"));
  Serial.println(F("   FAST WORKSPACE CALIBRATION ROUTINE  "));
  Serial.println(F("    Bottom-Left -> Top-Right Corner    "));
  Serial.println(F("========================================\n"));
  
  // 1. Find bottom-left corner (X-min, Y-min)
  Serial.println(F("Finding bottom-left corner..."));
  home_x_min_sensorless();
  delay(300);
  home_y_min_sensorless();
  delay(300);
  
  // 2. Find top-right corner (one axis at a time to avoid speed mismatch)
  Serial.println(F("\nFinding top-right corner..."));
  home_x_max_sensorless();
  delay(300);
  home_y_max_sensorless();
  delay(300);
  
  // 3. Return to origin (bottom-left, one axis at a time)
  Serial.println(F("\nReturning to origin..."));
  home_x_min_sensorless();
  delay(300);
  home_y_min_sensorless();
  delay(300);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("   WORKSPACE CALIBRATION COMPLETE!    "));
  Serial.println(F("   Origin at bottom-left corner (0,0) "));
  Serial.println(F("========================================\n"));
}

// ---------------- Circle demo function -------------------

void run_circle_once() {
  Serial.println(F("\n=== Running circle path ==="));

  // Move to starting point (rightmost point of circle)
  float start_x = CIRCLE_CENTER_X_MM + CIRCLE_RADIUS_MM;
  float start_y = CIRCLE_CENTER_Y_MM;
  
  Serial.print(F("Moving to circle start: ("));
  Serial.print(start_x);
  Serial.print(F(", "));
  Serial.print(start_y);
  Serial.println(F(")"));
  
  move_to_xy_mm(start_x, start_y);
  delay(500);

  // Execute circle
  Serial.println(F("Tracing circle..."));
  
  for (int i = 0; i <= CIRCLE_SEGMENTS; i++) {
    float angle = (2.0f * PI * i) / CIRCLE_SEGMENTS;
    float x = CIRCLE_CENTER_X_MM + CIRCLE_RADIUS_MM * cosf(angle);
    float y = CIRCLE_CENTER_Y_MM + CIRCLE_RADIUS_MM * sinf(angle);
    
    move_to_xy_mm(x, y);
  }

  Serial.println(F("Circle complete!\n"));
}

// ------------------------ SETUP/LOOP ---------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println();
  Serial.println(F("CoreXY + 2x TMC2209 sensorless homing + circle demo (RP2350)"));

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

  if (!drivers_ok) {
    Serial.println(F("Calibration skipped because UART check failed."));
    return;
  }

  Serial.println(F("Starting workspace calibration in 3 seconds..."));
  delay(3000);

  calibrate_workspace();

  Serial.println(F("Starting circle demo in 2 seconds..."));
  delay(2000);
}

void loop() {
  if (!drivers_ok) {
    delay(1000);
    return;
  }

  run_circle_once();
  delay(1000);   // pause between circles and repeat
}