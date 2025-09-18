#include <Arduino.h>
#include <STM32LowPower.h>
#include <Notecard.h>
#include "data_mode.h"

Notecard notecard;
DataMode dataMode;

void setup() {
  // Configure LED pin
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize the low power library
  LowPower.begin();
  notecard.begin();

  // Configure Notecard with Product UID and periodic sync
  J *req = notecard.newRequest("hub.set");
  if (req != NULL) {
    JAddStringToObject(req, "product", "com.gmail.taulabtech:taulabtest");
    JAddStringToObject(req, "mode", "periodic");
    JAddNumberToObject(req, "outbound", 5);
    JAddNumberToObject(req, "inbound", 60);
    notecard.sendRequest(req);
  }

  // Initialize LSM6DSOX sensor (don't auto-start logging)
  dataMode.begin(&notecard);

  // Stop any auto-started logging to control it manually
  if (dataMode.getIsLogging()) {
    dataMode.stopLogging();
  }
}

void loop() {
  // LED on during data collection
  digitalWrite(LED_BUILTIN, HIGH);

  // Ensure we start fresh
  if (dataMode.getIsLogging()) {
    dataMode.stopLogging();
  }

  // Start new logging session
  dataMode.startLogging();

  // Run data collection until it completes (dataMode handles the 10-second timer internally)
  while (dataMode.getIsLogging()) {
    dataMode.update();
    delay(10); // Minimal delay - let dataMode control timing
  }

  // LED off after collection completes
  digitalWrite(LED_BUILTIN, LOW);

  // Sleep for 3 minutes
  LowPower.deepSleep(180000); // 3 minutes = 180000ms
}