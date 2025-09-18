#include "gps_tracker.h"

GPSTracker::GPSTracker() : notecard(nullptr), initialized(false), lastRequest(0) {
}

bool GPSTracker::begin(Notecard* nc) {
    notecard = nc;
    if (notecard == nullptr) {
        return false;
    }

    Serial.println("=== GPS TRACKER INITIALIZED ===");
    // Don't configure GPS mode here - leave it off for power saving
    // GPS will be enabled only when actually requesting location

    initialized = true;
    return true;
}

GPSData GPSTracker::getLocation() {
    GPSData result = {false, 0.0, 0.0, 0.0, 0.0, 0, 0, ""};

    if (!initialized || notecard == nullptr) {
        result.error = "GPS tracker not initialized";
        return result;
    }

    // Just request location - don't change GPS mode (leave it off for power saving)

    // Request current location
    J *req = notecard->newRequest("card.location");
    if (req == NULL) {
        result.error = "Failed to create location request";
        return result;
    }

    J *rsp = notecard->requestAndResponse(req);
    if (rsp == NULL) {
        result.error = "No response from Notecard";
        return result;
    }

    // Parse the response
    result = parseLocationResponse(rsp);

    // Clean up
    notecard->deleteResponse(rsp);

    return result;
}

bool GPSTracker::sendLocationToCloud(const GPSData& gpsData) {
    if (!initialized || notecard == nullptr) {
        Serial.println("Cannot send GPS data - tracker not initialized");
        return false;
    }

    if (!gpsData.valid) {
        Serial.println("Cannot send invalid GPS data");
        return false;
    }

    Serial.println("Sending GPS location to Notehub...");

    // Create JSON note with GPS data
    J *req = notecard->newRequest("note.add");
    JAddStringToObject(req, "file", "data.qo");
    JAddBoolToObject(req, "sync", true);

    J *body = JAddObjectToObject(req, "body");
    if (body) {
        JAddNumberToObject(body, "latitude", gpsData.latitude);
        JAddNumberToObject(body, "longitude", gpsData.longitude);
        JAddNumberToObject(body, "altitude", gpsData.altitude);
        JAddNumberToObject(body, "hdop", gpsData.hdop);
        JAddNumberToObject(body, "satellites", gpsData.satellites);
        JAddNumberToObject(body, "timestamp", gpsData.timestamp);
        JAddStringToObject(body, "data_type", "gps_location");
    }

    bool success = notecard->sendRequest(req);

    if (success) {
        Serial.println("✓ GPS location sent to Notehub successfully");
    } else {
        Serial.println("✗ Failed to send GPS location to Notehub");
    }

    return success;
}

GPSData GPSTracker::parseLocationResponse(J* rsp) {
    GPSData result = {false, 0.0, 0.0, 0.0, 0.0, 0, 0, ""};

    if (rsp == NULL) {
        result.error = "Null response";
        return result;
    }

    // Check for errors
    if (JHasObjectItem(rsp, "err")) {
        result.error = JGetString(rsp, "err");
        return result;
    }

    // Check if location data is available
    if (!JHasObjectItem(rsp, "lat") || !JHasObjectItem(rsp, "lon")) {
        result.error = "No location data in response";
        return result;
    }

    // Extract location data
    result.latitude = JGetNumber(rsp, "lat");
    result.longitude = JGetNumber(rsp, "lon");

    // Optional fields
    if (JHasObjectItem(rsp, "alt")) {
        result.altitude = JGetNumber(rsp, "alt");
    }

    if (JHasObjectItem(rsp, "hdop")) {
        result.hdop = JGetNumber(rsp, "hdop");
    }

    if (JHasObjectItem(rsp, "sats")) {
        result.satellites = JGetNumber(rsp, "sats");
    }

    if (JHasObjectItem(rsp, "time")) {
        result.timestamp = JGetNumber(rsp, "time");
    } else {
        result.timestamp = millis();
    }

    // Validate coordinates
    if (result.latitude >= -90.0 && result.latitude <= 90.0 &&
        result.longitude >= -180.0 && result.longitude <= 180.0) {
        result.valid = true;
    } else {
        result.error = "Invalid coordinates";
    }

    return result;
}