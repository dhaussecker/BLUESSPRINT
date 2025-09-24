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
uint8_t previousMlcState = 0;
int interruptOccurred = 0;  // Track if any interrupts happened during cycle

// Interrupt Service Routine
void onWakePin() {
    wokeByPin = true;
}

// Read current MLC state from data mode
uint8_t getCurrentMlcState() {
    return dataMode.getCurrentMlcState();
}

// Add state event to the log
void addStateEvent(unsigned long startTime, unsigned long endTime, uint8_t mlcState) {
    if (stateEventCount < MAX_STATE_EVENTS) {
        stateEvents[stateEventCount].startTime = startTime;
        stateEvents[stateEventCount].endTime = endTime;
        stateEvents[stateEventCount].stateLog = mlcState;  // Use provided MLC state
        stateEventCount++;

    } else {
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

        // Mark that an interrupt occurred this cycle
        interruptOccurred = 1;

        // Get current MLC state to check for actual state change
        uint8_t currentMlcState = getCurrentMlcState();

        // Only log if state actually changed
        if (lastStateTime > 0 && currentMlcState != previousMlcState) {
            // Log the previous state (from lastStateTime to current time)
            addStateEvent(lastStateTime, currentTime, previousMlcState);

            // Update previous state and last state time for next transition
            previousMlcState = currentMlcState;
            lastStateTime = currentTime;
        }

    }
}


void setup() {
  // Configure LED pin
  pinMode(LED_BUILTIN, OUTPUT);

  // Configure interrupt pin
  pinMode(WAKE_PIN, INPUT_PULLDOWN);

  // Initialize the low power library
  LowPower.begin();

  // Initialize RTC
  rtc.begin();

  notecard.begin();

  // Disable Notecard debug output to save power
  notecard.setDebugOutputStream(0);

  // Configure Notecard with Product UID and periodic sync
  J *req = notecard.newRequest("hub.set");
  if (req != NULL) {
    JAddStringToObject(req, "product", "com.gmail.taulabtech:taulabtest");
    JAddStringToObject(req, "mode", "periodic");
    JAddStringToObject(req, "voutbound", "usb:60;high:60;normal:120;low:360;dead:0");
    JAddStringToObject(req, "vinbound", "usb:1440;high:1440;normal:2880;low:10080;dead:0");
    notecard.sendRequest(req);
  }
    // Configure GPS location mode
  req = notecard.newRequest("card.location.mode");
  if (req != NULL) {
    JAddStringToObject(req, "mode", "periodic");
    JAddStringToObject(req, "vseconds", "usb:1800;high:1800;normal:1800;low:86400;dead:0");
    notecard.sendRequest(req);
  }
  // Enable location tracking
  req = notecard.newRequest("card.location.track");
  if (req != NULL) {
    JAddBoolToObject(req, "start", true);
    JAddBoolToObject(req, "heartbeat", true);
    JAddNumberToObject(req, "hours", 24);
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
}

void loop() {
  // START IN DATA MODE - Run once only
  if (!dataModeDone) {

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

    // Immediately send sensors.qo with Format 1
    collectMode.sendData(); // This sends to sensors.qo with acceleration data

    dataModeDone = true;
  }

  // COLLECT MODE - Repeating cycle

  // Calibrate system time by retrieving UTC timestamp
  TimestampResult result = collectMode.getNotecardTimestamp();

  if (result.success && result.unixTime > 0) {
    storedUTCTimestamp = result.unixTime;
    collectMode.storeTimestamp(result.unixTime);

    // Set RTC to the actual UTC time (like EXAMPLECODE)
    rtc.setEpoch(result.unixTime);

    // Initialize state logging - preserve continuity across cycles
    // Only reset on very first initialization (when stateEventCount is 0)
    if (stateEventCount == 0 && lastStateTime == 0) {
        // First-time initialization
        lastStateTime = result.unixTime;
        previousMlcState = getCurrentMlcState();
    }
    // Always reset interrupt counter for new cycle
    interruptOccurred = 0;


  } else {
    delay(5000);
    return; // Try again
  }

  // Enter DEEP SLEEP for 30 minutes (with interrupt wake capability)

  // Sleep with interrupt handling loop - FIXED TIMING
  unsigned long cycleStartTime = storedUTCTimestamp; // Original cycle start time
  unsigned long targetWakeTime = cycleStartTime + 1800; // 30 minutes from CYCLE START


  while (true) {
    unsigned long currentTime = rtc.isTimeSet() ? rtc.getEpoch() : 0;

    // Calculate remaining sleep time from ORIGINAL cycle start
    if (currentTime >= targetWakeTime) {
      break; // 30 minutes elapsed since cycle start
    }

    unsigned long remainingSeconds = targetWakeTime - currentTime;
    unsigned long remainingSleepMS = remainingSeconds * 1000;


    // Deep sleep for remaining time (or max 30 minutes)
    unsigned long sleepTime = remainingSleepMS > 1800000 ? 1800000 : remainingSleepMS;
    LowPower.deepSleep(sleepTime);

    // Check if we woke by interrupt
    if (wokeByPin) {
      handleInterruptWake();

      // Continue loop to check remaining time
      continue;
    } else {
      // Woke by timer - check if full 30 minutes elapsed
      unsigned long newCurrentTime = rtc.isTimeSet() ? rtc.getEpoch() : 0;
      if (newCurrentTime >= targetWakeTime) {
        break;
      }
      // If not, continue sleeping
    }
  }

  // Wake up by timer expiry - normal cycle

  // Check if any interrupts occurred during this 30-minute cycle
  if (interruptOccurred == 0) {
    return; // Go back to sleep immediately - huge power savings!
  }


  // Get current RTC time (should be ~30 minutes after stored time)
  unsigned long currentRTCTime = 0;
  if (rtc.isTimeSet()) {
    currentRTCTime = rtc.getEpoch();
  } else {
    currentRTCTime = storedUTCTimestamp + 1800; // Fallback: assume 30 minutes passed
  }

  // Add final state event with current state lasting until now
  // Only add if we have accumulated time in the current state
  if (lastStateTime > 0 && lastStateTime < currentRTCTime) {
    addStateEvent(lastStateTime, currentRTCTime, previousMlcState);
  }

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

  // Disable interrupts during critical cleanup to prevent race conditions
  noInterrupts();

  // CRITICAL: Clear state array after transmission to prevent duplicates
  // Preserve final state timing for continuity
  if (stateEventCount > 0) {
    // Get the final state's end time and state for next cycle continuity
    lastStateTime = stateEvents[stateEventCount - 1].endTime;
    previousMlcState = stateEvents[stateEventCount - 1].stateLog;
  }

  // Clear the state array and reset counter
  stateEventCount = 0;

  // Re-enable interrupts
  interrupts();

  // Loop back to COLLECT MODE (ENTER HERE point)
}