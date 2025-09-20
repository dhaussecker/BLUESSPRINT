#include <Arduino.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>
#include <Notecard.h>
#include "data_mode.h"
#include "collect_mode.h"

STM32RTC& rtc = STM32RTC::getInstance();
Notecard notecard;
DataMode dataMode;
CollectMode collectMode;

// Variables for flow control
bool dataModeDone = false;
unsigned long storedUTCTimestamp = 0;


void setup() {
  // Configure LED pin
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize the low power library
  LowPower.begin();

  // Initialize RTC
  rtc.begin();
  Serial.println("RTC initialized");

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

  // Initialize collect mode with data_mode reference
  collectMode.begin(&notecard, &dataMode);

  // Stop any auto-started logging to control it manually
  if (dataMode.getIsLogging()) {
    dataMode.stopLogging();
  }
}

void loop() {
  // START IN DATA MODE - Run once only
  if (!dataModeDone) {
    Serial.println("🚀 STARTING DATA MODE");
    Serial.println("📊 Logging accelerometer samples for fixed duration...");

    digitalWrite(LED_BUILTIN, HIGH);

    if (dataMode.getIsLogging()) {
      dataMode.stopLogging();
    }

    dataMode.startLogging();

    while (dataMode.getIsLogging()) {
      dataMode.update();
      delay(10);
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("✅ Accelerometer data collection complete");

    // Immediately send sensors.qo with Format 1
    Serial.println("📤 Immediately sending sensors.qo...");
    collectMode.sendData(); // This sends to sensors.qo with acceleration data

    dataModeDone = true;
    Serial.println("✅ DATA MODE COMPLETE - Switching to COLLECT MODE");
  }

  // COLLECT MODE - Repeating cycle
  Serial.println("🔄 COLLECT MODE");

  // Calibrate system time by retrieving UTC timestamp
  Serial.println("🕐 Calibrating system time...");
  TimestampResult result = collectMode.getNotecardTimestamp();

  if (result.success && result.unixTime > 0) {
    storedUTCTimestamp = result.unixTime;
    collectMode.storeTimestamp(result.unixTime);

    // Set RTC to the actual UTC time (like EXAMPLECODE)
    rtc.setEpoch(result.unixTime);
    Serial.println("🕐 RTC synchronized with Notecard time");
    Serial.print("✅ UTC timestamp stored locally: ");
    Serial.println(storedUTCTimestamp);
  } else {
    Serial.println("❌ Failed to get timestamp, retrying in 5 seconds...");
    delay(5000);
    return; // Try again
  }

  // Enter DEEP SLEEP for 3 minutes
  Serial.println("💤 ENTERING DEEP SLEEP for 3 minutes");
  LowPower.deepSleep(180000); // 3 minutes

  // Wake up automatically at timer expiry
  Serial.println("🔄 WAKE UP - Sending data.qo with Format 2");

  // Get current RTC time (should be ~3 minutes after stored time)
  unsigned long currentRTCTime = 0;
  if (rtc.isTimeSet()) {
    currentRTCTime = rtc.getEpoch();
    Serial.print("🕐 Current RTC time: ");
    Serial.println(currentRTCTime);
    Serial.print("⏱️  Expected ~3 minutes later than: ");
    Serial.println(storedUTCTimestamp);
  } else {
    Serial.println("❌ RTC not set, using fallback");
    currentRTCTime = storedUTCTimestamp + 180; // Fallback: assume 3 minutes passed
  }

  // Send data.qo with Format 2 (statelog entries)
  collectMode.sendStateLog(storedUTCTimestamp, currentRTCTime);

  Serial.println("🔄 REPEATING COLLECT MODE CYCLE...");
  // Loop back to COLLECT MODE (ENTER HERE point)
}