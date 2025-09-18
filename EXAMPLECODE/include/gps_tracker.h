#ifndef GPS_TRACKER_H
#define GPS_TRACKER_H

#include <Arduino.h>
#include <Notecard.h>

struct GPSData {
    bool valid;
    double latitude;
    double longitude;
    double altitude;
    float hdop;
    int satellites;
    unsigned long timestamp;
    String error;
};

class GPSTracker {
private:
    Notecard* notecard;
    bool initialized;
    unsigned long lastRequest;
    const unsigned long GPS_TIMEOUT = 30000; // 30 second timeout

public:
    GPSTracker();

    bool begin(Notecard* nc);
    GPSData getLocation();
    bool sendLocationToCloud(const GPSData& gpsData);

private:
    GPSData parseLocationResponse(J* rsp);
};

#endif // GPS_TRACKER_H