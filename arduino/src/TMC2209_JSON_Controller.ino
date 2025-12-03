#include <TMCStepper.h>
#include <ArduinoJson.h>

// ---- USER CONFIG ----
#define R_SENSE 0.11f

// XIAO ESP32-S3 UART pins for TMC2209 bus
#define TMC_UART_RX D10  // Right motor RX (unused in your pinout but kept for TMC)
#define TMC_UART_TX D9   // Right motor TX (unused in your pinout but kept for TMC)

// Right Motor pins
#define STEP1_PIN D5     // Right STEP
#define DIR1_PIN D2      // Right DIR
#define EN1_PIN D7       // Right ENABLE

// Left Motor pins
#define STEP2_PIN D1     // Left STEP
#define DIR2_PIN D0      // Left DIR
#define EN2_PIN D6       // Left ENABLE

// TMC2209 slave addresses
#define DRIVER1_ADDR 0b00
#define DRIVER2_ADDR 0b01

// ---- MOTOR CONSTANTS ----
const float FULL_STEPS_PER_REV = 400.0f;  // 0.9° per step
const float MICROSTEPS = 64.0f;
const float MICROSTEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS;

// ---- MOTOR STATE ----
float target_rpm1 = 0.0f;  // Right motor
float target_rpm2 = 0.0f;  // Left motor
bool motorsEnabled = false;

const unsigned int STEP_PULSE_WIDTH_US = 4;

// Computed edge intervals
unsigned long stepInterval1_us = 0;
unsigned long stepInterval2_us = 0;

// Timing state for motor 1
unsigned long lastEdge1 = 0;
unsigned long lastPulseStart1 = 0;
bool pulseHigh1 = false;

// Timing state for motor 2
unsigned long lastEdge2 = 0;
unsigned long lastPulseStart2 = 0;
bool pulseHigh2 = false;

// ---- TMC objects ----
HardwareSerial TMCSerial(1);
TMC2209Stepper driver1(&TMCSerial, R_SENSE, DRIVER1_ADDR);
TMC2209Stepper driver2(&TMCSerial, R_SENSE, DRIVER2_ADDR);

// Command buffer
String commandBuffer = "";
const size_t JSON_BUFFER_SIZE = 512;

// Convert RPM -> edge interval in microseconds
unsigned long rpmToIntervalUs(float rpm) {
  float abs_rpm = fabs(rpm);
  if (abs_rpm < 1e-6f) return 0;

  float steps_per_sec = abs_rpm * MICROSTEPS_PER_REV / 60.0f;
  float edges_per_sec = steps_per_sec * 2.0f;

  return (unsigned long)(1000000.0f / edges_per_sec);
}

void computeIntervalsFromRPM() {
  stepInterval1_us = rpmToIntervalUs(target_rpm1);
  stepInterval2_us = rpmToIntervalUs(target_rpm2);

  digitalWrite(DIR1_PIN, (target_rpm1 >= 0.0f) ? HIGH : LOW);
  digitalWrite(DIR2_PIN, (target_rpm2 >= 0.0f) ? HIGH : LOW);
}

void sendResponse(const char* cmd, const char* status, const char* message) {
  JsonDocument doc;
  doc["cmd"] = cmd;
  doc["status"] = status;
  doc["message"] = message;
  serializeJson(doc, Serial);
  Serial.println();
}

void sendMotorStatus() {
  JsonDocument doc;
  doc["cmd"] = "status";
  doc["status"] = "ok";
  doc["motors_enabled"] = motorsEnabled;
  doc["motor1_rpm"] = target_rpm1;
  doc["motor2_rpm"] = target_rpm2;
  doc["motor1_current"] = driver1.rms_current();
  doc["motor2_current"] = driver2.rms_current();
  serializeJson(doc, Serial);
  Serial.println();
}

void handlePingCommand() {
  sendResponse("ping", "ok", "pong");
}

void handleEnableCommand(JsonDocument& doc) {
  bool state = doc["state"] | true;
  const char* motor = doc["motor"] | "both";
  
  if (strcmp(motor, "R") == 0 || strcmp(motor, "right") == 0) {
    digitalWrite(EN1_PIN, state ? LOW : HIGH);
    sendResponse("enable", "ok", state ? "Right motor enabled" : "Right motor disabled");
  }
  else if (strcmp(motor, "L") == 0 || strcmp(motor, "left") == 0) {
    digitalWrite(EN2_PIN, state ? LOW : HIGH);
    sendResponse("enable", "ok", state ? "Left motor enabled" : "Left motor disabled");
  }
  else {
    digitalWrite(EN1_PIN, state ? LOW : HIGH);
    digitalWrite(EN2_PIN, state ? LOW : HIGH);
    motorsEnabled = state;
    if (!state) {
      target_rpm1 = 0.0f;
      target_rpm2 = 0.0f;
      computeIntervalsFromRPM();
    }
    sendResponse("enable", "ok", state ? "Motors enabled" : "Motors disabled");
  }
}

void handleMoveCommand(JsonDocument& doc) {
  if (!motorsEnabled) {
    sendResponse("move", "error", "Motors not enabled");
    return;
  }

  const char* motor = doc["motor"] | "both";
  
  if (strcmp(motor, "R") == 0 || strcmp(motor, "right") == 0) {
    target_rpm1 = doc["rpm"] | 0.0f;
    computeIntervalsFromRPM();
    sendResponse("move", "ok", "Right motor set");
  }
  else if (strcmp(motor, "L") == 0 || strcmp(motor, "left") == 0) {
    target_rpm2 = doc["rpm"] | 0.0f;
    computeIntervalsFromRPM();
    sendResponse("move", "ok", "Left motor set");
  }
  else if (strcmp(motor, "both") == 0) {
    target_rpm1 = doc["r_rpm"] | 0.0f;
    target_rpm2 = doc["l_rpm"] | 0.0f;
    computeIntervalsFromRPM();
    sendResponse("move", "ok", "Both motors set");
  }
  else {
    sendResponse("move", "error", "Invalid motor");
  }
}

void handleStopCommand(JsonDocument& doc) {
  const char* motor = doc["motor"] | "both";
  
  if (strcmp(motor, "R") == 0 || strcmp(motor, "right") == 0) {
    target_rpm1 = 0.0f;
    computeIntervalsFromRPM();
    sendResponse("stop", "ok", "Right motor stopped");
  }
  else if (strcmp(motor, "L") == 0 || strcmp(motor, "left") == 0) {
    target_rpm2 = 0.0f;
    computeIntervalsFromRPM();
    sendResponse("stop", "ok", "Left motor stopped");
  }
  else {
    target_rpm1 = 0.0f;
    target_rpm2 = 0.0f;
    computeIntervalsFromRPM();
    sendResponse("stop", "ok", "All motors stopped");
  }
}

void handleSetCurrentCommand(JsonDocument& doc) {
  int current = doc["current"] | 800;
  const char* motor = doc["motor"] | "both";
  
  if (strcmp(motor, "R") == 0 || strcmp(motor, "right") == 0) {
    driver1.rms_current(current);
    sendResponse("setcurrent", "ok", "Right motor current set");
  }
  else if (strcmp(motor, "L") == 0 || strcmp(motor, "left") == 0) {
    driver2.rms_current(current);
    sendResponse("setcurrent", "ok", "Left motor current set");
  }
  else {
    driver1.rms_current(current);
    driver2.rms_current(current);
    sendResponse("setcurrent", "ok", "Both motors current set");
  }
}

void processCommand(String jsonCommand) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, jsonCommand);
  
  if (error) {
    sendResponse("error", "failed", "Invalid JSON");
    return;
  }
  
  const char* cmd = doc["cmd"];
  
  if (!cmd) {
    sendResponse("error", "failed", "Missing 'cmd' field");
    return;
  }
  
  // Command routing
  if (strcmp(cmd, "ping") == 0) {
    handlePingCommand();
  }
  else if (strcmp(cmd, "enable") == 0) {
    handleEnableCommand(doc);
  }
  else if (strcmp(cmd, "move") == 0) {
    handleMoveCommand(doc);
  }
  else if (strcmp(cmd, "stop") == 0) {
    handleStopCommand(doc);
  }
  else if (strcmp(cmd, "status") == 0) {
    sendMotorStatus();
  }
  else if (strcmp(cmd, "setcurrent") == 0) {
    handleSetCurrentCommand(doc);
  }
  else {
    sendResponse("error", "failed", "Unknown command");
  }
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (commandBuffer.length() > 0) {
        processCommand(commandBuffer);
        commandBuffer = "";
      }
    } else {
      commandBuffer += c;
      
      if (commandBuffer.length() > JSON_BUFFER_SIZE) {
        sendResponse("error", "failed", "Command buffer overflow");
        commandBuffer = "";
      }
    }
  }
}

void setup() {
  // USB Serial for JSON commands
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  // I/O setup
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(EN1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(EN2_PIN, OUTPUT);

  digitalWrite(EN1_PIN, HIGH);  // Disabled initially (active LOW)
  digitalWrite(EN2_PIN, HIGH);  // Disabled initially (active LOW)
  digitalWrite(STEP1_PIN, LOW);
  digitalWrite(STEP2_PIN, LOW);

  // TMC2209 UART
  TMCSerial.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);

  // Driver 1 config
  driver1.begin();
  driver1.toff(5);
  driver1.blank_time(24);
  driver1.rms_current(800);
  driver1.microsteps((uint16_t)MICROSTEPS);
  driver1.en_spreadCycle(false);
  driver1.pdn_disable(true);
  driver1.I_scale_analog(false);

  // Driver 2 config
  driver2.begin();
  driver2.toff(5);
  driver2.blank_time(24);
  driver2.rms_current(800);
  driver2.microsteps((uint16_t)MICROSTEPS);
  driver2.en_spreadCycle(false);
  driver2.pdn_disable(true);
  driver2.I_scale_analog(false);

  computeIntervalsFromRPM();
  
  Serial.println("{\"type\":\"info\",\"message\":\"TMC2209 JSON Controller Ready\"}");
  sendResponse("status", "ready", "Controller initialized");
}

void loop() {
  // Process serial commands
  processSerialCommands();
  
  unsigned long now = micros();

  // Motor 1 stepping
  if (motorsEnabled && stepInterval1_us > 0) {
    if (!pulseHigh1) {
      if (now - lastEdge1 >= stepInterval1_us) {
        digitalWrite(STEP1_PIN, HIGH);
        pulseHigh1 = true;
        lastPulseStart1 = now;
        lastEdge1 = now;
      }
    } else {
      if (now - lastPulseStart1 >= STEP_PULSE_WIDTH_US) {
        digitalWrite(STEP1_PIN, LOW);
        pulseHigh1 = false;
      }
    }
  }

  // Motor 2 stepping
  if (motorsEnabled && stepInterval2_us > 0) {
    if (!pulseHigh2) {
      if (now - lastEdge2 >= stepInterval2_us) {
        digitalWrite(STEP2_PIN, HIGH);
        pulseHigh2 = true;
        lastPulseStart2 = now;
        lastEdge2 = now;
      }
    } else {
      if (now - lastPulseStart2 >= STEP_PULSE_WIDTH_US) {
        digitalWrite(STEP2_PIN, LOW);
        pulseHigh2 = false;
      }
    }
  }
}
