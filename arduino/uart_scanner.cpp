/*
 * TMC2209 UART Address & Bus Scanner
 * 
 * Scans all possible UART configurations and addresses to find TMC2209 drivers.
 * Tests different bus combinations and all 4 possible addresses (0b00, 0b01, 0b10, 0b11)
 * 
 * This will definitively tell you which pins and addresses your drivers are on.
 */

#include <Arduino.h>
#include <TMCStepper.h>

#define R_SENSE 0.11f

// All possible pins on XIAO ESP32-S3
#define TEST_PIN_D0  D0
#define TEST_PIN_D1  D1
#define TEST_PIN_D2  D2
#define TEST_PIN_D3  D3
#define TEST_PIN_D4  D4
#define TEST_PIN_D5  D5
#define TEST_PIN_D6  D6
#define TEST_PIN_D7  D7
#define TEST_PIN_D8  D8
#define TEST_PIN_D9  D9
#define TEST_PIN_D10 D10

// Your suspected pins from config
struct PinPair {
  int rx;
  int tx;
  const char* name;
};

// Test these bus configurations
PinPair busCombos[] = {
  {D3, D4, "Bus1 (D3/D4)"},
  {D9, D10, "Bus2 (D9/D10)"},
  {D4, D3, "Bus1-Swapped (D4/D3)"},   // Try swapped
  {D10, D9, "Bus2-Swapped (D10/D9)"},  // Try swapped
};

const int NUM_BUSES = sizeof(busCombos) / sizeof(busCombos[0]);

// All possible TMC2209 addresses
const uint8_t addresses[] = {0b00, 0b01, 0b10, 0b11};
const char* addressNames[] = {"0b00", "0b01", "0b10", "0b11"};
const int NUM_ADDRESSES = 4;

HardwareSerial* testSerial1 = nullptr;
HardwareSerial* testSerial2 = nullptr;

void setup() {
  Serial.begin(115200);
  delay(3000);  // Wait for serial monitor
  
  Serial.println("\n\n╔════════════════════════════════════════════════╗");
  Serial.println("║   TMC2209 UART BUS & ADDRESS SCANNER          ║");
  Serial.println("╚════════════════════════════════════════════════╝\n");
  
  Serial.println("This tool will:");
  Serial.println("  1. Test all bus combinations (normal + swapped RX/TX)");
  Serial.println("  2. Try all 4 possible addresses on each bus");
  Serial.println("  3. Report which configurations respond\n");
  
  Serial.println("Testing configurations:");
  for (int i = 0; i < NUM_BUSES; i++) {
    Serial.print("  ");
    Serial.print(busCombos[i].name);
    Serial.print(" - RX: D");
    Serial.print(busCombos[i].rx);
    Serial.print(", TX: D");
    Serial.println(busCombos[i].tx);
  }
  Serial.println("\nStarting scan...\n");
  Serial.println("═══════════════════════════════════════════════\n");
  
  delay(1000);
  
  int foundCount = 0;
  
  // Test each bus configuration
  for (int busIdx = 0; busIdx < NUM_BUSES; busIdx++) {
    Serial.print("Testing ");
    Serial.print(busCombos[busIdx].name);
    Serial.println(":");
    Serial.println("───────────────────────────────────────────────");
    
    // Create serial instance for this bus
    HardwareSerial testSerial(busIdx + 1);  // Use UART1, UART2, etc.
    testSerial.begin(115200, SERIAL_8N1, busCombos[busIdx].rx, busCombos[busIdx].tx);
    delay(100);
    
    // Test each address on this bus
    for (int addrIdx = 0; addrIdx < NUM_ADDRESSES; addrIdx++) {
      uint8_t addr = addresses[addrIdx];
      
      Serial.print("  Address ");
      Serial.print(addressNames[addrIdx]);
      Serial.print(": ");
      
      // Create driver instance
      TMC2209Stepper driver(&testSerial, R_SENSE, addr);
      driver.begin();
      delay(50);
      
      // Try to read VERSION register
      uint8_t version = driver.version();
      delay(10);
      
      // Verify by reading again
      uint8_t version2 = driver.version();
      delay(10);
      
      if (version != 0 && version != 0xFF && version == version2) {
        // Valid response!
        Serial.print("✓ FOUND! Version: 0x");
        Serial.print(version, HEX);
        Serial.println();
        foundCount++;
        
        // Try to get more info
        delay(50);
        uint16_t current = driver.rms_current();
        uint16_t microsteps = driver.microsteps();
        
        Serial.print("    Current: ");
        Serial.print(current);
        Serial.print("mA, Microsteps: ");
        Serial.println(microsteps);
        
        // Test write capability
        driver.rms_current(600);
        delay(50);
        uint16_t readback = driver.rms_current();
        
        if (readback > 500 && readback < 700) {
          Serial.println("    Write test: ✓ PASSED");
        } else {
          Serial.print("    Write test: ✗ FAILED (wrote 600, read ");
          Serial.print(readback);
          Serial.println(")");
        }
        
      } else if (version == 0 || version == 0xFF) {
        Serial.println("✗ No response");
      } else {
        Serial.print("? Inconsistent (0x");
        Serial.print(version, HEX);
        Serial.print(" / 0x");
        Serial.print(version2, HEX);
        Serial.println(")");
      }
      
      delay(100);
    }
    
    testSerial.end();
    Serial.println();
    delay(200);
  }
  
  // Summary
  Serial.println("═══════════════════════════════════════════════");
  Serial.println("SCAN COMPLETE");
  Serial.println("═══════════════════════════════════════════════\n");
  
  if (foundCount == 0) {
    Serial.println("✗✗✗ NO DRIVERS FOUND ✗✗✗\n");
    Serial.println("Possible issues:");
    Serial.println("  1. Drivers not powered (check VIO = 3.3V, VM = 12-24V)");
    Serial.println("  2. PDN_UART not pulled HIGH");
    Serial.println("  3. Wrong pins - drivers not on D3/D4 or D9/D10");
    Serial.println("  4. Faulty drivers or connections");
    Serial.println("  5. Drivers in standalone mode (not UART mode)");
  } else if (foundCount == 1) {
    Serial.println("⚠ ONLY 1 DRIVER FOUND ⚠\n");
    Serial.println("Expected 2 drivers. Check the second driver:");
    Serial.println("  - Power connections");
    Serial.println("  - UART wiring");
    Serial.println("  - PDN_UART pin");
  } else if (foundCount == 2) {
    Serial.println("✓✓✓ BOTH DRIVERS FOUND ✓✓✓\n");
    Serial.println("Use the configurations shown above in your main code!");
  } else {
    Serial.println("⚠ Found more than 2 drivers - possible address conflicts!");
  }
  
  Serial.println("\n═══════════════════════════════════════════════");
  Serial.println("Continuous monitoring mode starting...");
  Serial.println("Will re-scan every 10 seconds");
  Serial.println("═══════════════════════════════════════════════\n");
}

void loop() {
  static unsigned long lastScan = 0;
  
  if (millis() - lastScan > 10000) {
    lastScan = millis();
    
    Serial.println("\n[Quick Re-scan]");
    
    // Quick scan with known good configs
    HardwareSerial bus1(1);
    bus1.begin(115200, SERIAL_8N1, D3, D4);
    delay(50);
    
    HardwareSerial bus2(2);
    bus2.begin(115200, SERIAL_8N1, D9, D10);
    delay(50);
    
    for (int addr = 0; addr < 4; addr++) {
      TMC2209Stepper driver1(&bus1, R_SENSE, addr);
      driver1.begin();
      delay(10);
      uint8_t v1 = driver1.version();
      
      if (v1 != 0 && v1 != 0xFF) {
        Serial.print("Bus1 addr ");
        Serial.print(addr);
        Serial.print(": ✓ (0x");
        Serial.print(v1, HEX);
        Serial.println(")");
      }
      
      TMC2209Stepper driver2(&bus2, R_SENSE, addr);
      driver2.begin();
      delay(10);
      uint8_t v2 = driver2.version();
      
      if (v2 != 0 && v2 != 0xFF) {
        Serial.print("Bus2 addr ");
        Serial.print(addr);
        Serial.print(": ✓ (0x");
        Serial.print(v2, HEX);
        Serial.println(")");
      }
    }
    
    bus1.end();
    bus2.end();
  }
  
  delay(100);
}
