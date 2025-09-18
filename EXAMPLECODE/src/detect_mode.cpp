#include "detect_mode.h"
#include "notecard_time.h"
#include "movement.h"
#include <Wire.h>

DetectMode::DetectMode(Notecard& nc, STM32RTC& rtcInstance)
    : notecard(nc), rtc(rtcInstance), currentStage(STAGE_TIMESTAMP_COLLECTION),
      timestampCollected(false), stageStartTime(0), eventCount(0),
      lastStateTime(0), lastMlcState(0), accelerometer(nullptr), accelInitialized(false),
      lastTransmissionTime(0) {
}

void DetectMode::begin() {
    Serial.println("=== DETECT MODE INITIALIZED ===");
    currentStage = STAGE_TIMESTAMP_COLLECTION;
    timestampCollected = false;
    stageStartTime = millis();

    // Initialize GPS tracker
    gpsTracker.begin(&notecard);

    // Initialize state storage
    eventCount = 0;
    lastStateTime = 0;
    lastMlcState = 0;
    lastTransmissionTime = 0;
}

void DetectMode::update() {
    switch (currentStage) {
        case STAGE_TIMESTAMP_COLLECTION:
            handleStage1();
            break;
        case STAGE_2:
            handleStage2();
            break;
    }
}

DetectStage DetectMode::getCurrentStage() {
    return currentStage;
}

void DetectMode::handleStage1() {
    static bool stageMessageShown = false;

    if (!stageMessageShown) {
        Serial.println("=== STAGE 1: TIMESTAMP COLLECTION ===");
        Serial.println("Attempting to collect timestamp from Notecard...");
        stageMessageShown = true;
    }

    // Try to get timestamp every 5 seconds
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt > 5000) {
        Serial.println("Requesting timestamp from Notecard...");

        TimestampResult result = getNotecardTimestamp(notecard);

        if (result.success) {
            Serial.print("✓ TIMESTAMP COLLECTED SUCCESSFULLY! Unix time: ");
            Serial.println(result.unixTime);

            // Set RTC with the timestamp
            rtc.setEpoch(result.unixTime);
            Serial.println("RTC synchronized with Notecard time");

            timestampCollected = true;

            // Initialize accelerometer for state detection
            if (initializeAccelerometer()) {
                Serial.println("Accelerometer initialized for state detection");

                // Store initial state after timestamp collection
                storeStateEvent("timestamp_sync", result.unixTime);
            } else {
                Serial.println("Failed to initialize accelerometer - continuing without state detection");
            }

            // Move to Stage 2
            Serial.println("Moving to STAGE 2...");
            currentStage = STAGE_2;
            stageStartTime = millis();
            stageMessageShown = false; // Reset for next stage
        } else {
            Serial.println("✗ Failed to get timestamp, retrying...");
        }

        lastAttempt = millis();
    }
}

void DetectMode::handleStage2() {
    static bool stageMessageShown = false;

    if (!stageMessageShown) {
        Serial.println("=== STAGE 2: DEEP SLEEP MODE ===");
        Serial.println("Entering deep sleep - wake on interrupt or 2min timer");
        Serial.println("Visual indicators:");
        Serial.println("  Timer wake: 1 long blink (2 seconds)");
        Serial.println("  Interrupt wake: 3 fast blinks (handled separately)");
        Serial.print("Current RTC time before sleep: ");
        if (rtc.isTimeSet()) {
            Serial.println(rtc.getEpoch());
        } else {
            Serial.println("NOT SET");
        }
        delay(100); // Allow serial to flush
        stageMessageShown = true;
    }

    // Enter deep sleep for 2 minutes (120,000 ms) - Notecard stays sleeping
    LowPower.deepSleep(120000);

    // We're awake from TIMER (if interrupt occurred, it would be handled in main loop)
    Serial.println("=== WOKE UP FROM TIMER ===");
    Serial.print("Current RTC time: ");
    unsigned long currentTime = 0;
    if (rtc.isTimeSet()) {
        currentTime = rtc.getEpoch();
        Serial.println(currentTime);
    } else {
        Serial.println("NOT SET");
        currentTime = millis() / 1000; // Fallback to millis-based timestamp
    }

    // Store timer wake state event
    if (accelInitialized) {
        storeStateEvent("timer_wake", currentTime);

        // Check if it's time to send data
        if (shouldSendData()) {
            addFinalStateAndSend();
        }
    }

    // Skip GPS for now to test pure low-power operation
    // Notecard should stay sleeping

    // Visual indication for timer wake: 1 long blink
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);  // 2 second long blink
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);   // Brief pause before next sleep cycle
}

void DetectMode::handleWakeInterrupt() {
    Serial.println("=== WOKE UP FROM INTERRUPT ===");
    Serial.print("Current RTC time: ");
    unsigned long currentTime = 0;
    if (rtc.isTimeSet()) {
        currentTime = rtc.getEpoch();
        Serial.println(currentTime);
    } else {
        Serial.println("NOT SET");
        currentTime = millis() / 1000; // Fallback to millis-based timestamp
    }

    // Store interrupt wake state event
    if (accelInitialized) {
        storeStateEvent("interrupt_wake", currentTime);
    }

    // Visual indication for interrupt wake: 3 fast blinks
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);  // Fast blink
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }

    Serial.println("Interrupt handled, returning to sleep cycle");
}

bool DetectMode::initializeAccelerometer() {
    Serial.println("Initializing LSM6DSOX accelerometer for state detection...");

    // Initialize I2C if not already done
    Wire.begin();
    Wire.setClock(400000);

    // Create accelerometer instance
    accelerometer = new LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

    // Initialize the sensor
    if (accelerometer->begin() != LSM6DSOX_OK) {
        Serial.println("Failed to initialize LSM6DSOX library");
        return false;
    }

    Serial.println("LSM6DSOX library initialized successfully");

    // Enable accelerometer for MLC
    if (accelerometer->Enable_X() != LSM6DSOX_OK) {
        Serial.println("Failed to enable accelerometer");
        return false;
    }

    // Set low power configuration for long-term monitoring
    if (accelerometer->Set_X_ODR(26.0f) != LSM6DSOX_OK) {
        Serial.println("Failed to set accelerometer ODR");
        return false;
    }

    if (accelerometer->Set_X_FS(2) != LSM6DSOX_OK) {
        Serial.println("Failed to set accelerometer full scale");
        return false;
    }

    Serial.println("Accelerometer configured: 26Hz, ±2g");

    // Load MLC configuration for motion detection
    Serial.println("Loading MLC configuration for state detection...");

    ucf_line_t *ProgramPointer = (ucf_line_t *)movement;
    int32_t TotalNumberOfLine = sizeof(movement) / sizeof(ucf_line_t);

    for (int32_t LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++) {
        if (accelerometer->Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
            Serial.print("Error loading MLC program at line: ");
            Serial.println(LineCounter);
            return false;
        }
    }

    Serial.println("MLC program loaded successfully");
    delay(100); // Allow sensor to stabilize

    accelInitialized = true;
    return true;
}

uint8_t DetectMode::readMlcState() {
    if (!accelInitialized || accelerometer == nullptr) {
        return 0;
    }

    uint8_t mlc_out[8];
    if (accelerometer->Get_MLC_Output(mlc_out) == LSM6DSOX_OK) {
        return mlc_out[0]; // Return first MLC output
    }
    return 0;
}

void DetectMode::storeStateEvent(const String& eventType, unsigned long startTime, unsigned long endTime) {
    if (eventCount >= MAX_STATE_EVENTS) {
        Serial.println("State event buffer full, sending to cloud...");
        sendStateEventsToCloud();
        eventCount = 0; // Reset after sending
    }

    uint8_t currentMlcState = readMlcState();

    stateEvents[eventCount].mlcState = currentMlcState;
    stateEvents[eventCount].startTime = startTime;
    stateEvents[eventCount].endTime = (endTime == 0) ? rtc.getEpoch() : endTime;
    stateEvents[eventCount].eventType = eventType;

    Serial.print("Stored state event: ");
    Serial.print(eventType);
    Serial.print(", MLC State: ");
    Serial.print(currentMlcState);
    Serial.print(", Time: ");
    Serial.println(stateEvents[eventCount].startTime);

    eventCount++;
    lastMlcState = currentMlcState;
    lastStateTime = rtc.getEpoch();
}

void DetectMode::sendStateEventsToCloud() {
    if (eventCount == 0) {
        Serial.println("No state events to send");
        return;
    }

    Serial.print("Sending ");
    Serial.print(eventCount);
    Serial.println(" state events to cloud...");

    // Create JSON note with state events
    J *req = notecard.newRequest("note.add");
    JAddStringToObject(req, "file", "states.qo");
    JAddBoolToObject(req, "sync", true);

    J *body = JAddObjectToObject(req, "body");
    if (body) {
        JAddNumberToObject(body, "event_count", eventCount);
        JAddStringToObject(body, "data_type", "accel_state_events");

        // Create array of events
        J *events = JAddArrayToObject(body, "events");
        for (int i = 0; i < eventCount; i++) {
            J *event = JCreateObject();
            JAddNumberToObject(event, "mlc_state", stateEvents[i].mlcState);
            JAddNumberToObject(event, "start_time", stateEvents[i].startTime);
            JAddNumberToObject(event, "end_time", stateEvents[i].endTime);
            JAddStringToObject(event, "event_type", stateEvents[i].eventType.c_str());
            JAddItemToArray(events, event);
        }
    }

    bool success = notecard.sendRequest(req);

    if (success) {
        Serial.println("✓ State events sent to cloud successfully");
        eventCount = 0; // Reset after successful send
        lastTransmissionTime = rtc.isTimeSet() ? rtc.getEpoch() : millis() / 1000;
    } else {
        Serial.println("✗ Failed to send state events to cloud");
    }
}

bool DetectMode::shouldSendData() {
    if (eventCount >= MAX_STATE_EVENTS) {
        Serial.println("Event buffer full - triggering transmission");
        return true;
    }

    unsigned long currentTime = rtc.isTimeSet() ? rtc.getEpoch() : millis() / 1000;

    if (lastTransmissionTime == 0) {
        // First transmission after initialization
        lastTransmissionTime = currentTime;
        return false;
    }

    if ((currentTime - lastTransmissionTime) >= TRANSMISSION_INTERVAL) {
        Serial.print("Transmission interval reached (");
        Serial.print(TRANSMISSION_INTERVAL);
        Serial.println("s) - triggering transmission");
        return true;
    }

    return false;
}

void DetectMode::addFinalStateAndSend() {
    Serial.println("=== PREPARING DATA TRANSMISSION ===");

    // Store final state before transmission
    unsigned long currentTime = rtc.isTimeSet() ? rtc.getEpoch() : millis() / 1000;
    storeStateEvent("final_capture", currentTime);

    // Send all collected data to cloud
    sendStateEventsToCloud();

    Serial.println("=== DATA TRANSMISSION COMPLETE ===");
}