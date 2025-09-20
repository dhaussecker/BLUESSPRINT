#include "collect_mode.h"
#include "data_mode.h"

CollectMode::CollectMode() : notecard(nullptr), dataMode(nullptr), storedTimestamp(0), hasStoredTimestamp(false) {
}

bool CollectMode::begin(Notecard* nc, DataMode* dm) {
    notecard = nc;
    dataMode = dm;
    return (notecard != nullptr && dataMode != nullptr);
}

TimestampResult CollectMode::getNotecardTimestamp() {
    TimestampResult result = {0, false};

    if (notecard == nullptr) {
        Serial.println("ERROR: Notecard not initialized in CollectMode");
        return result;
    }

    Serial.println("🕐 COLLECT_MODE: Requesting timestamp from Notecard...");

    // Request current time from Notecard
    J *req = notecard->newRequest("card.time");
    if (req == NULL) {
        Serial.println("❌ Failed to create card.time request");
        return result;
    }

    // Get response
    J *rsp = notecard->requestAndResponse(req);
    if (rsp == NULL) {
        Serial.println("❌ Failed to get response from card.time");
        return result;
    }

    // Check for time field in response
    if (JHasObjectItem(rsp, "time")) {
        result.unixTime = JGetNumber(rsp, "time");
        result.success = true;
        Serial.print("✅ TIMESTAMP COLLECTED! Unix time: ");
        Serial.println(result.unixTime);
    } else {
        Serial.println("❌ No time field in Notecard response");
    }

    // Clean up response
    notecard->deleteResponse(rsp);

    return result;
}

void CollectMode::storeTimestamp(unsigned long timestamp) {
    storedTimestamp = timestamp;
    hasStoredTimestamp = (timestamp > 0);
    Serial.print("📦 TIMESTAMP STORED: ");
    Serial.println(storedTimestamp);
}

unsigned long CollectMode::getStoredTimestamp() {
    return storedTimestamp;
}

bool CollectMode::hasValidStoredTimestamp() {
    return hasStoredTimestamp && (storedTimestamp > 0);
}

void CollectMode::sendData() {
    if (!hasValidStoredTimestamp()) {
        Serial.println("⚠️  No stored timestamp, skipping data send");
        return;
    }

    Serial.println("📤 SENDING DATA with stored timestamp...");
    sendAccelerationData();

    // Clear stored timestamp after sending
    hasStoredTimestamp = false;
    storedTimestamp = 0;
    Serial.println("✅ Data sent, timestamp cleared");
}

void CollectMode::sendAccelerationData() {
    if (dataMode == nullptr) {
        Serial.println("❌ DataMode not initialized");
        return;
    }

    int samples = dataMode->getCollectedSamples();
    if (samples <= 0) {
        Serial.println("📭 No acceleration data to send");
        return;
    }

    Serial.print("📊 Sending ");
    Serial.print(samples);
    Serial.print(" acceleration samples with timestamp: ");
    Serial.println(storedTimestamp);

    // Get data arrays from data_mode
    float* ax_samples = dataMode->getAxSamples();
    float* ay_samples = dataMode->getAySamples();
    float* az_samples = dataMode->getAzSamples();

    // Calculate total size needed
    int total_size = samples * 12;  // 3 floats * 4 bytes each

    // Create buffer with all data
    uint8_t* all_data = (uint8_t*)malloc(total_size);
    if (all_data == NULL) {
        Serial.println("❌ Failed to allocate memory for data");
        return;
    }

    // Pack all samples into the buffer
    for (int i = 0; i < samples; i++) {
        int offset = i * 12;
        memcpy(&all_data[offset], &ax_samples[i], 4);
        memcpy(&all_data[offset + 4], &ay_samples[i], 4);
        memcpy(&all_data[offset + 8], &az_samples[i], 4);
    }

    // Base64 encode the entire dataset
    int encodedLen = ((total_size + 2) / 3) * 4 + 1;
    char* encoded = (char*)malloc(encodedLen);
    if (encoded == NULL) {
        Serial.println("❌ Failed to allocate memory for encoded data");
        free(all_data);
        return;
    }

    JB64Encode(encoded, (const char*)all_data, total_size);

    // Send as regular JSON note with base64 data
    J *req = notecard->newRequest("note.add");
    JAddStringToObject(req, "file", "sensors.qo");
    JAddBoolToObject(req, "sync", true);

    J *body = JAddObjectToObject(req, "body");
    if (body) {
        JAddStringToObject(body, "data", encoded);
        JAddNumberToObject(body, "samples", samples);
        JAddNumberToObject(body, "format", 1);  // 1 = float32 ax,ay,az format
        JAddNumberToObject(body, "rate_hz", dataMode->getCurrentODR());
        JAddNumberToObject(body, "duration_ms", dataMode->getLoggingDuration());
        JAddNumberToObject(body, "timestamp", storedTimestamp); // Using stored UTC timestamp
    }

    bool success = notecard->sendRequest(req);

    if (success) {
        Serial.println("✅ Successfully sent acceleration data as base64 JSON note");
    } else {
        Serial.println("❌ Failed to send acceleration data");
    }

    // Clean up
    free(all_data);
    free(encoded);
}

void CollectMode::sendTimestampOnly() {
    if (!hasValidStoredTimestamp()) {
        Serial.println("⚠️  No stored timestamp, skipping timestamp send");
        return;
    }

    Serial.print("🕐 SENDING TIMESTAMP ONLY: ");
    Serial.println(storedTimestamp);

    // Send just timestamp as JSON note - Format 2
    J *req = notecard->newRequest("note.add");
    JAddStringToObject(req, "file", "data.qo");
    JAddBoolToObject(req, "sync", true);

    J *body = JAddObjectToObject(req, "body");
    if (body) {
        JAddNumberToObject(body, "UTCTIMESTAMP", storedTimestamp);
    }

    bool success = notecard->sendRequest(req);

    if (success) {
        Serial.println("✅ Successfully sent timestamp-only note");
    } else {
        Serial.println("❌ Failed to send timestamp");
    }

    // Clear stored timestamp after sending
    hasStoredTimestamp = false;
    storedTimestamp = 0;
    Serial.println("🗑️  Timestamp cleared");
}

void CollectMode::sendStateLog(unsigned long utcTimestamp, unsigned long currentRTCTime) {
    Serial.println("📊 SENDING STATELOG FORMAT 2");
    Serial.print("Start time: ");
    Serial.print(utcTimestamp);
    Serial.print(", End time: ");
    Serial.println(currentRTCTime);

    // Use the timestamps directly
    unsigned long startTime = utcTimestamp;    // When we started data logging
    unsigned long endTime = currentRTCTime;    // Current RTC time (3 minutes later)

    // Send Format 2 with statelog entries
    J *req = notecard->newRequest("note.add");
    JAddStringToObject(req, "file", "data.qo");
    JAddBoolToObject(req, "sync", true);

    J *body = JAddObjectToObject(req, "body");
    if (body) {
        // Create simple statelog array with format: [statelog, timestamp, timestamp]
        J *entries = JCreateArray();

        // Entry 1: Data logging start (statelog: 0, start_time, start_time)
        J *entry1 = JCreateArray();
        JAddItemToArray(entry1, JCreateNumber(0));           // statelog: 0
        JAddItemToArray(entry1, JCreateNumber(startTime));   // timestamp
        JAddItemToArray(entry1, JCreateNumber(startTime));   // timestamp again

        // Entry 2: Data logging end (statelog: 0, end_time, end_time)
        J *entry2 = JCreateArray();
        JAddItemToArray(entry2, JCreateNumber(0));           // statelog: 0
        JAddItemToArray(entry2, JCreateNumber(endTime));     // timestamp
        JAddItemToArray(entry2, JCreateNumber(endTime));     // timestamp again

        JAddItemToArray(entries, entry1);
        JAddItemToArray(entries, entry2);

        JAddItemToObject(body, "entries", entries);
    }

    bool success = notecard->sendRequest(req);

    if (success) {
        Serial.println("✅ Successfully sent statelog data.qo");
        Serial.print("📅 Start: ");
        Serial.print(startTime);
        Serial.print(" End: ");
        Serial.print(endTime);
        Serial.print(" Duration: ");
        Serial.print(currentRTCTime);
        Serial.println(" seconds");
    } else {
        Serial.println("❌ Failed to send statelog");
    }
}