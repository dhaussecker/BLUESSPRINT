#include <Arduino.h>
#include <STM32LowPower.h>
#include <STM32RTC.h>
#include <Notecard.h>
#include "data_mode.h"
#include "collect_mode.h"

// D6 interrupt pin for state detection
const int WAKE_PIN = D6;
volatile bool wokeByPin = false;

STM32RTC& rtc = STM32RTC::getInstance();
Notecard notecard;
DataMode dataMode;
CollectMode collectMode;

// Variables for flow control
bool dataModeDone = false;
unsigned long storedUTCTimestamp = 0;

// State logging system
#define MAX_STATE_EVENTS 50
struct StateEvent {
    unsigned long startTime;
    unsigned long endTime;
    int stateLog;
};
StateEvent stateEvents[MAX_STATE_EVENTS];
int stateEventCount = 0;
unsigned long lastStateTime = 0;

// Interrupt Service Routine
void onWakePin() {
    wokeByPin = true;
}

// Read current MLC state from data mode
uint8_t getCurrentMlcState() {
    return dataMode.getCurrentMlcState();
}

// Add state event to the log
void addStateEvent(unsigned long startTime, unsigned long endTime) {
    if (stateEventCount < MAX_STATE_EVENTS) {
        stateEvents[stateEventCount].startTime = startTime;
        stateEvents[stateEventCount].endTime = endTime;
        stateEvents[stateEventCount].stateLog = getCurrentMlcState();  // Use actual MLC state
        stateEventCount++;

        Serial.print("📝 State event added: ");
        Serial.print(startTime);
        Serial.print(" → ");
        Serial.print(endTime);
        Serial.print(" (MLC State: ");
        Serial.print(stateEvents[stateEventCount-1].stateLog);
        Serial.println(")");
    } else {
        Serial.println("⚠️  State event buffer full!");
    }
}

// Handle interrupt wake - log state transition
void handleInterruptWake() {
    if (wokeByPin) {
        wokeByPin = false;  // Clear flag

        // Quick double blink to indicate interrupt detected
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);

        unsigned long currentTime = rtc.isTimeSet() ? rtc.getEpoch() : 0;
        Serial.print("🔔 INTERRUPT TRIGGERED at RTC time: ");
        Serial.println(currentTime);

        if (lastStateTime > 0) {
            // Log the previous state (from lastStateTime to current time)
            addStateEvent(lastStateTime, currentTime);
        }

        // Update last state time for next transition
        lastStateTime = currentTime;

        Serial.println("💤 Going back to deep sleep immediately");
    }
}


void setup() {
  // Configure LED pin
  pinMode(LED_BUILTIN, OUTPUT);

  // Configure interrupt pin
  pinMode(WAKE_PIN, INPUT_PULLDOWN);
  Serial.println("D6 interrupt pin configured");

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

  // Attach interrupt for wake from deep sleep
  LowPower.attachInterruptWakeup(WAKE_PIN, onWakePin, RISING);
  Serial.println("Interrupt attached for D6 RISING edge");
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

    // Initialize state logging - add START event
    lastStateTime = result.unixTime;
    stateEventCount = 0;  // Reset state events for new cycle

    // Add START event (same time twice)
    addStateEvent(result.unixTime, result.unixTime);
    Serial.print("🏁 START event added: ");
    Serial.println(lastStateTime);

    Serial.print("✅ UTC timestamp stored locally: ");
    Serial.println(storedUTCTimestamp);
  } else {
    Serial.println("❌ Failed to get timestamp, retrying in 5 seconds...");
    delay(5000);
    return; // Try again
  }

  // Enter DEEP SLEEP for 3 minutes (with interrupt wake capability)
  Serial.println("💤 ENTERING DEEP SLEEP for 3 minutes (interrupt on D6 enabled)");

  // Sleep with interrupt handling loop - FIXED TIMING
  unsigned long cycleStartTime = storedUTCTimestamp; // Original cycle start time
  unsigned long targetWakeTime = cycleStartTime + 180; // 3 minutes from CYCLE START

  Serial.print("🕐 Cycle start: ");
  Serial.print(cycleStartTime);
  Serial.print(", Target wake: ");
  Serial.println(targetWakeTime);

  while (true) {
    unsigned long currentTime = rtc.isTimeSet() ? rtc.getEpoch() : 0;

    // Calculate remaining sleep time from ORIGINAL cycle start
    if (currentTime >= targetWakeTime) {
      Serial.println("⏰ 3-minute cycle complete - breaking out of sleep");
      break; // 3 minutes elapsed since cycle start
    }

    unsigned long remainingSeconds = targetWakeTime - currentTime;
    unsigned long remainingSleepMS = remainingSeconds * 1000;

    Serial.print("💤 Sleeping for ");
    Serial.print(remainingSeconds);
    Serial.println(" more seconds until 3-minute cycle complete");

    // Deep sleep for remaining time (or max 3 minutes)
    unsigned long sleepTime = remainingSleepMS > 180000 ? 180000 : remainingSleepMS;
    LowPower.deepSleep(sleepTime);

    // Check if we woke by interrupt
    if (wokeByPin) {
      Serial.println("🔔 WOKE BY INTERRUPT - Handling state change");
      handleInterruptWake();

      // Continue loop to check remaining time
      continue;
    } else {
      // Woke by timer - check if full 3 minutes elapsed
      unsigned long newCurrentTime = rtc.isTimeSet() ? rtc.getEpoch() : 0;
      if (newCurrentTime >= targetWakeTime) {
        Serial.println("⏰ 3-minute timer completed");
        break;
      }
      // If not, continue sleeping
    }
  }

  // Wake up by timer expiry - normal cycle
  Serial.println("⏰ WAKE UP BY TIMER - Sending data.qo with Format 2");

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

  // Add END event (same time twice)
  addStateEvent(currentRTCTime, currentRTCTime);
  Serial.print("🔚 END event added: ");
  Serial.println(currentRTCTime);

  // Extract arrays from StateEvent structs
  unsigned long startTimes[MAX_STATE_EVENTS];
  unsigned long endTimes[MAX_STATE_EVENTS];
  int stateLogs[MAX_STATE_EVENTS];

  for (int i = 0; i < stateEventCount; i++) {
    startTimes[i] = stateEvents[i].startTime;
    endTimes[i] = stateEvents[i].endTime;
    stateLogs[i] = stateEvents[i].stateLog;
  }

  // Send data.qo with Format 2 (all state events)
  collectMode.sendAllStateEvents(startTimes, endTimes, stateLogs, stateEventCount);

  Serial.println("🔄 REPEATING COLLECT MODE CYCLE...");
  // Loop back to COLLECT MODE (ENTER HERE point)
}