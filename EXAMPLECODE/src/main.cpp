#include <Arduino.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>
#include <Notecard.h>
#include "detect_mode.h"
#include "data_mode.h"

// Pick an EXTI-capable pin wired to your sensor/button.
// For a simple test, wire the pin to GND through a momentary switch.
const int WAKE_PIN = D6;   // change to your actual interrupt pin

volatile bool wokeByPin = false;

STM32RTC& rtc = STM32RTC::getInstance();
Notecard notecard;
DetectMode detectMode(notecard, rtc);
DataMode dataMode;

// ============================================
// CHANGE THIS ANYWHERE IN THE PROGRAM:
// 0 = COLLECT MODE (timestamp + deep sleep)
// 1 = DATA MODE (accelerometer readings)
// ============================================
int currentMode = 1;

// Mode initialization flags
bool collectModeInitialized = false;
bool dataModeInitialized = false;

const uint32_t SLEEP_MS = 30000;  // sleep 30 seconds between prints

void onWakePin() {
  // This runs in interrupt context right after wake.
  wokeByPin = true;
}

void setup() {
  // Optional: give USB serial time to open before first sleep
  Serial.begin(115200);
  delay(2500);

  // Initialize hardware
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(WAKE_PIN, INPUT_PULLDOWN); // active-low interrupt to GND

  rtc.begin();
  LowPower.begin();

  // Initialize Notecard
  notecard.begin();
  notecard.setDebugOutputStream(Serial);

  // Wait for Notecard to be ready
  delay(2000);

  // Force Notecard into deep sleep (most aggressive approach)
  {
    J *req = notecard.newRequest("card.attn");
    if (req != NULL) {
      JAddStringToObject(req, "mode", "sleep");
      JAddBoolToObject(req, "start", true);
      JAddNumberToObject(req, "seconds", -1);  // Sleep indefinitely
      notecard.sendRequest(req);
      Serial.println("Notecard forced into deep sleep");
    }
  }

  delay(1000); // Give command time to take effect

  // Wake on falling edge of WAKE_PIN
  LowPower.attachInterruptWakeup(WAKE_PIN, onWakePin, RISING);

  Serial.println("=== DUAL MODE SYSTEM ===");
  Serial.println("MODE 0: COLLECT MODE (timestamp + deep sleep)");
  Serial.println("MODE 1: DATA MODE (accelerometer readings)");
  Serial.println("Change 'currentMode' variable to switch modes");
  Serial.println("Setup complete");
}

void loop() {
  // Handle wake pin interrupt if it occurred
  if (wokeByPin) {
    wokeByPin = false;

    if (currentMode == 0) {
      // COLLECT MODE interrupt handling
      if (collectModeInitialized && detectMode.getCurrentStage() == STAGE_2) {
        detectMode.handleWakeInterrupt();
      } else {
        Serial.println("Wake pin interrupt in COLLECT MODE");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
      }
    } else if (currentMode == 1) {
      // DATA MODE interrupt handling
      Serial.println("Wake pin interrupt in DATA MODE");
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  // Switch modes based on currentMode variable
  if (currentMode == 0) {
    // COLLECT MODE
    if (!collectModeInitialized) {
      Serial.println("=== INITIALIZING COLLECT MODE ===");
      detectMode.begin();
      collectModeInitialized = true;
      dataModeInitialized = false;
    }
    detectMode.update();

    // Only delay in STAGE 1 (STAGE 2 handles its own timing via sleep)
    if (detectMode.getCurrentStage() == STAGE_TIMESTAMP_COLLECTION) {
      delay(100);
    }

  } else if (currentMode == 1) {
    // DATA MODE
    if (!dataModeInitialized) {
      Serial.println("=== INITIALIZING DATA MODE ===");
      if (dataMode.begin(&notecard)) {
        dataMode.setModePointer(&currentMode); // Pass currentMode pointer
        dataModeInitialized = true;
        collectModeInitialized = false;
      } else {
        Serial.println("Failed to initialize DATA MODE");
        return;
      }
    }
    dataMode.update();
    // No delay in DATA MODE - let data_mode handle its own timing
  }
}