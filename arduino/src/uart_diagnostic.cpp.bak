/*
 * TMC2209 UART Diagnostic Test
 * 
 * This program tests UART communication with TMC2209 drivers
 * and helps identify if issues are software or hardware related.
 * 
 * Upload this to your ESP32-S3 to diagnose UART problems.
 */

#include <Arduino.h>
#include <TMCStepper.h>

// Pin definitions - same as your main controller
#define R_SENSE 0.11f

// UART pins
#define TMC1_UART_RX D3
#define TMC1_UART_TX D4
#define TMC2_UART_RX D9
#define TMC2_UART_TX D10

// Control pins (not needed for UART test but good to verify)
#define STEP1_PIN D5
#define DIR1_PIN D2
#define EN1_PIN D7
#define STEP2_PIN D1
#define DIR2_PIN D0
#define EN2_PIN D6

#define DRIVER1_ADDR 0b00
#define DRIVER2_ADDR 0b00

HardwareSerial TMCSerial1(1);
HardwareSerial TMCSerial2(2);
TMC2209Stepper driver1(&TMCSerial1, R_SENSE, DRIVER1_ADDR);
TMC2209Stepper driver2(&TMCSerial2, R_SENSE, DRIVER2_ADDR);

void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for serial monitor
  
  Serial.println("\n\n========================================");
  Serial.println("TMC2209 UART DIAGNOSTIC TEST");
  Serial.println("========================================\n");
  
  // Setup control pins
  pinMode(EN1_PIN, OUTPUT);
  pinMode(EN2_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  
  digitalWrite(EN1_PIN, HIGH);  // Disabled
  digitalWrite(EN2_PIN, HIGH);  // Disabled
  
  Serial.println("Step 1: Pin Configuration");
  Serial.println("-------------------------");
  Serial.print("Driver 1 UART: RX=D");
  Serial.print(TMC1_UART_RX);
  Serial.print(", TX=D");
  Serial.println(TMC1_UART_TX);
  Serial.print("Driver 2 UART: RX=D");
  Serial.print(TMC2_UART_RX);
  Serial.print(", TX=D");
  Serial.println(TMC2_UART_TX);
  Serial.println("✓ Pins configured\n");
  
  // Initialize UART buses
  Serial.println("Step 2: Initialize UART Buses");
  Serial.println("-----------------------------");
  TMCSerial1.begin(115200, SERIAL_8N1, TMC1_UART_RX, TMC1_UART_TX);
  Serial.println("✓ UART1 initialized (115200 baud)");
  
  TMCSerial2.begin(115200, SERIAL_8N1, TMC2_UART_RX, TMC2_UART_TX);
  Serial.println("✓ UART2 initialized (115200 baud)\n");
  
  delay(500);
  
  // Initialize TMC drivers
  Serial.println("Step 3: Initialize TMC2209 Drivers");
  Serial.println("-----------------------------------");
  driver1.begin();
  Serial.println("✓ Driver 1 begin() called");
  
  driver2.begin();
  Serial.println("✓ Driver 2 begin() called\n");
  
  delay(500);
  
  // Test UART communication
  Serial.println("Step 4: Test UART Communication");
  Serial.println("--------------------------------");
  
  Serial.println("\n[DRIVER 1 TESTS]");
  testDriver(driver1, "Driver 1");
  
  Serial.println("\n[DRIVER 2 TESTS]");
  testDriver(driver2, "Driver 2");
  
  // Summary
  Serial.println("\n========================================");
  Serial.println("DIAGNOSTIC SUMMARY");
  Serial.println("========================================\n");
  
  // Overall assessment
  bool driver1_ok = testDriverQuick(driver1);
  bool driver2_ok = testDriverQuick(driver2);
  
  if (driver1_ok && driver2_ok) {
    Serial.println("✓✓✓ BOTH DRIVERS RESPONDING ✓✓✓");
    Serial.println("\nUART is working! Your issue is likely:");
    Serial.println("  - Software logic error");
    Serial.println("  - Configuration problem");
    Serial.println("  - Command/response handling");
  }
  else if (!driver1_ok && !driver2_ok) {
    Serial.println("✗✗✗ NO DRIVERS RESPONDING ✗✗✗");
    Serial.println("\nHardware issue detected. Check:");
    Serial.println("  1. Driver power (VIO, VM)");
    Serial.println("  2. PDN_UART pin pulled HIGH");
    Serial.println("  3. MS1/MS2 pins to GND (address 0b00)");
    Serial.println("  4. RX/TX wiring (are they crossed?)");
    Serial.println("  5. Common ground between ESP32 and drivers");
  }
  else {
    Serial.println("⚠ PARTIAL FAILURE ⚠");
    if (driver1_ok) {
      Serial.println("Driver 1: ✓ Working");
      Serial.println("Driver 2: ✗ Not responding");
    } else {
      Serial.println("Driver 1: ✗ Not responding");
      Serial.println("Driver 2: ✓ Working");
    }
    Serial.println("\nCheck the non-working driver:");
    Serial.println("  - Power connections");
    Serial.println("  - UART TX/RX wiring");
    Serial.println("  - PDN_UART and address pins");
  }
  
  Serial.println("\n========================================");
  Serial.println("Test complete. Monitor will continue...");
  Serial.println("========================================\n");
}

void testDriver(TMC2209Stepper &driver, const char* name) {
  Serial.print("\nTesting ");
  Serial.print(name);
  Serial.println(":");
  Serial.println("------------------");
  
  // Test 1: Read version register
  Serial.print("  1. Reading VERSION register... ");
  uint8_t version = driver.version();
  if (version == 0 || version == 0xFF) {
    Serial.print("✗ FAIL (got 0x");
    Serial.print(version, HEX);
    Serial.println(")");
    Serial.println("     → UART not responding!");
  } else {
    Serial.print("✓ OK (0x");
    Serial.print(version, HEX);
    Serial.println(")");
  }
  
  delay(100);
  
  // Test 2: Write and read current
  Serial.print("  2. Setting RMS current to 500mA... ");
  driver.rms_current(500);
  delay(100);
  uint16_t current = driver.rms_current();
  if (current > 400 && current < 600) {
    Serial.print("✓ OK (read back ");
    Serial.print(current);
    Serial.println("mA)");
  } else {
    Serial.print("✗ FAIL (read back ");
    Serial.print(current);
    Serial.println("mA)");
    Serial.println("     → UART write/read mismatch!");
  }
  
  delay(100);
  
  // Test 3: Microstep configuration
  Serial.print("  3. Setting microsteps to 16... ");
  driver.microsteps(16);
  delay(100);
  uint16_t msteps = driver.microsteps();
  if (msteps == 16) {
    Serial.println("✓ OK");
  } else {
    Serial.print("✗ FAIL (read back ");
    Serial.print(msteps);
    Serial.println(")");
  }
  
  delay(100);
  
  // Test 4: Read GSTAT (status register)
  Serial.print("  4. Reading GSTAT register... ");
  uint8_t gstat = driver.GSTAT();
  Serial.print("✓ Read: 0x");
  Serial.println(gstat, HEX);
  
  delay(100);
  
  // Test 5: Enable/disable test
  Serial.print("  5. Testing enable state... ");
  driver.toff(4);  // Enable driver
  delay(50);
  uint8_t toff = driver.toff();
  if (toff == 4) {
    Serial.println("✓ OK");
  } else {
    Serial.print("✗ FAIL (toff=");
    Serial.print(toff);
    Serial.println(")");
  }
}

bool testDriverQuick(TMC2209Stepper &driver) {
  uint8_t version = driver.version();
  return (version != 0 && version != 0xFF);
}

void loop() {
  // Continuously monitor UART health
  static unsigned long lastTest = 0;
  
  if (millis() - lastTest > 5000) {  // Every 5 seconds
    lastTest = millis();
    
    Serial.println("\n[Periodic UART Health Check]");
    
    uint8_t v1 = driver1.version();
    uint8_t v2 = driver2.version();
    
    Serial.print("Driver 1 version: 0x");
    Serial.print(v1, HEX);
    Serial.print(" - ");
    Serial.println((v1 != 0 && v1 != 0xFF) ? "✓ OK" : "✗ FAIL");
    
    Serial.print("Driver 2 version: 0x");
    Serial.print(v2, HEX);
    Serial.print(" - ");
    Serial.println((v2 != 0 && v2 != 0xFF) ? "✓ OK" : "✗ FAIL");
    
    // Try reading current settings
    uint16_t i1 = driver1.rms_current();
    uint16_t i2 = driver2.rms_current();
    
    Serial.print("Driver 1 current: ");
    Serial.print(i1);
    Serial.println("mA");
    Serial.print("Driver 2 current: ");
    Serial.print(i2);
    Serial.println("mA");
  }
}
