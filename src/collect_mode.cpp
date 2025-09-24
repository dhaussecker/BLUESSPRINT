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
        return result;
    }


    // Request current time from Notecard
    J *req = notecard->newRequest("card.time");
    if (req == NULL) {
        return result;
    }

    // Get response
    J *rsp = notecard->requestAndResponse(req);
    if (rsp == NULL) {
        return result;
    }

    // Check for time field in response
    if (JHasObjectItem(rsp, "time")) {
        result.unixTime = JGetNumber(rsp, "time");
        result.success = true;
    } else {
    }

    // Clean up response
    notecard->deleteResponse(rsp);

    return result;
}

void CollectMode::storeTimestamp(unsigned long timestamp) {
    storedTimestamp = timestamp;
    hasStoredTimestamp = (timestamp > 0);
}

unsigned long CollectMode::getStoredTimestamp() {
    return storedTimestamp;
}

bool CollectMode::hasValidStoredTimestamp() {
    return hasStoredTimestamp && (storedTimestamp > 0);
}

void CollectMode::sendData() {
    if (!hasValidStoredTimestamp()) {
        return;
    }

    sendAccelerationData();

    // Clear stored timestamp after sending
    hasStoredTimestamp = false;
    storedTimestamp = 0;
}

void CollectMode::sendAccelerationData() {
    if (dataMode == nullptr) {
        return;
    }

    int samples = dataMode->getCollectedSamples();
    if (samples <= 0) {
        return;
    }


    // Get data arrays from data_mode
    float* ax_samples = dataMode->getAxSamples();
    float* ay_samples = dataMode->getAySamples();
    float* az_samples = dataMode->getAzSamples();

    // Calculate total size needed
    int total_size = samples * 12;  // 3 floats * 4 bytes each

    // Create buffer with all data
    uint8_t* all_data = (uint8_t*)malloc(total_size);
    if (all_data == NULL) {
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
    } else {
    }

    // Clean up
    free(all_data);
    free(encoded);
}

void CollectMode::sendTimestampOnly() {
    if (!hasValidStoredTimestamp()) {
        return;
    }


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
    } else {
    }

    // Clear stored timestamp after sending
    hasStoredTimestamp = false;
    storedTimestamp = 0;
}

void CollectMode::sendStateLog(unsigned long utcTimestamp, unsigned long currentRTCTime) {

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
    } else {
    }
}

void CollectMode::sendAllStateEvents(unsigned long* startTimes, unsigned long* endTimes, int* stateLogs, int eventCount) {

    if (eventCount == 0) {
        return;
    }

    // Send Format 2 with all state events
    J *req = notecard->newRequest("note.add");
    JAddStringToObject(req, "file", "data.qo");
    JAddBoolToObject(req, "sync", true);

    J *body = JAddObjectToObject(req, "body");
    if (body) {
        // Create entries array with all state events
        J *entries = JCreateArray();

        for (int i = 0; i < eventCount; i++) {
            // Entry format: [statelog, startTime, endTime]
            J *entry = JCreateArray();
            JAddItemToArray(entry, JCreateNumber(stateLogs[i]));
            JAddItemToArray(entry, JCreateNumber(startTimes[i]));
            JAddItemToArray(entry, JCreateNumber(endTimes[i]));
            JAddItemToArray(entries, entry);

        }

        JAddItemToObject(body, "entries", entries);
    }

    bool success = notecard->sendRequest(req);

    if (success) {
    } else {
    }
}