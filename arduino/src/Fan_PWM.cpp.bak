#include <Arduino.h>

int FAN_pin = D6;       // Fan connected to digital pin D6
int SOLENOID_pin = D4;  // Solenoid connected to digital pin D4
int IR_SENSOR_PIN7 = D7;  // IR sensor on pin 7 (puck release trigger)
int IR_SENSOR_PIN8 = D8;  // IR sensor on pin 8 (goal 1)
int IR_SENSOR_PIN9 = D9;  // IR sensor on pin 9 (goal 2)

bool lastSensor7State = HIGH;  // Track previous state to detect transitions
bool lastSensor8State = HIGH;  // Track goal 1 sensor
bool lastSensor9State = HIGH;  // Track goal 2 sensor

bool sensor7Triggered = false;  // Track if D7 has already been triggered
bool sensor8Triggered = false;  // Track if D8 has already been triggered
bool sensor9Triggered = false;  // Track if D9 has already been triggered

// bool puckDispensed = false;  // Track if puck has been dispensed (locked until goal scored)

void setup() {
  Serial.begin(115200);
  pinMode(FAN_pin, OUTPUT);
  pinMode(SOLENOID_pin, OUTPUT);
  pinMode(IR_SENSOR_PIN7, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN8, INPUT_PULLUP);
  pinMode(IR_SENSOR_PIN9, INPUT_PULLUP);
  
  analogWrite(FAN_pin, 0);  // Start with fan off
  digitalWrite(SOLENOID_pin, LOW);  // Solenoid off initially
  
  Serial.println("Fan & Solenoid Controller Ready");
  Serial.println("IR Sensors: D7 (puck trigger), D8 (goal 1), D9 (goal 2)");
  Serial.println("Commands:");
  Serial.println("  fan <0-255> - set fan speed (e.g., 'fan 128')");
  Serial.println("  fan off - turn fan off");
  Serial.println("  fan on - turn fan full speed");
  Serial.println("  pulse - trigger solenoid pulse (100ms)");
}

void loop() {
  // Read and print IR sensor data
  int sensor7_value = digitalRead(IR_SENSOR_PIN7);
  int sensor8_value = digitalRead(IR_SENSOR_PIN8);
  int sensor9_value = digitalRead(IR_SENSOR_PIN9);
  
  Serial.print("IR D7: ");
  Serial.print(sensor7_value);
  Serial.print(" | D8 (Goal1): ");
  Serial.print(sensor8_value);
  Serial.print(" | D9 (Goal2): ");
  Serial.println(sensor9_value);
  
  // Detect IR beam break on D7 (puck release trigger)
  if (sensor7_value == LOW && lastSensor7State == HIGH && !sensor7Triggered) {
    Serial.println("Coin detected! Starting fan and triggering solenoid...");
    
    // Turn on fan at full speed
    analogWrite(FAN_pin, 255);
    Serial.println("Fan turned on");
    
    // Trigger solenoid
    digitalWrite(SOLENOID_pin, HIGH);
    delay(100);  // 100ms pulse
    digitalWrite(SOLENOID_pin, LOW);
    Serial.println("Solenoid pulse complete - puck dispensed");
    sensor7Triggered = true;  // Mark as triggered
  }
  // Reset trigger when sensor goes back to HIGH (beam restored)
  if (sensor7_value == HIGH && lastSensor7State == LOW) {
    sensor7Triggered = false;  // Ready for next trigger
  }
  
  // Detect goal scored on D8 (Goal 1)
  if (sensor8_value == LOW && lastSensor8State == HIGH && !sensor8Triggered) {
    Serial.println("GOAL 1 SCORED! Stopping gantry and resetting...");
    Serial.println("STOP");  // Send stop command
    delay(500);  // Brief pause
    Serial.println("RESET");  // Send reset command
    // puckDispensed = false;  // Unlock solenoid - ready for next coin
    Serial.println("Ready for next coin insertion");
    sensor8Triggered = true;  // Mark as triggered
  }
  // Reset trigger when sensor goes back to HIGH (beam restored)
  if (sensor8_value == HIGH && lastSensor8State == LOW) {
    sensor8Triggered = false;  // Ready for next goal
  }
  
  // Detect goal scored on D9 (Goal 2)
  if (sensor9_value == LOW && lastSensor9State == HIGH && !sensor9Triggered) {
    Serial.println("GOAL 2 SCORED! Stopping gantry and resetting...");
    Serial.println("STOP");  // Send stop command
    delay(500);  // Brief pause
    Serial.println("RESET");  // Send reset command
    // puckDispensed = false;  // Unlock solenoid - ready for next coin
    Serial.println("Ready for next coin insertion");
    sensor9Triggered = true;  // Mark as triggered
  }
  // Reset trigger when sensor goes back to HIGH (beam restored)
  if (sensor9_value == HIGH && lastSensor8State == LOW) {
    sensor9Triggered = false;  // Ready for next goal
  }
  
  lastSensor7State = sensor7_value;
  lastSensor8State = sensor8_value;
  lastSensor9State = sensor9_value;
  
  delay(50);  // Read sensors every 50ms
  
  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("fan ")) {
      String speedStr = command.substring(4);
      
      if (speedStr == "off") {
        analogWrite(FAN_pin, 0);
        Serial.println("Fan turned off");
      }
      else if (speedStr == "on") {
        analogWrite(FAN_pin, 255);
        Serial.println("Fan at full speed");
      }
      else {
        int speed = speedStr.toInt();
        if (speed >= 0 && speed <= 255) {
          analogWrite(FAN_pin, speed);
          Serial.print("Fan speed set to: ");
          Serial.println(speed);
        }
        else {
          Serial.println("Error: Fan speed must be 0-255");
        }
      }
    }
    else if (command == "pulse") {
      // Trigger solenoid pulse
      digitalWrite(SOLENOID_pin, HIGH);
      Serial.println("Solenoid pulse started");
      delay(100);  // 100ms pulse
      digitalWrite(SOLENOID_pin, LOW);
      Serial.println("Solenoid pulse complete");
    }
  }
}
