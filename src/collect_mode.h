#ifndef COLLECT_MODE_H
#define COLLECT_MODE_H

#include <Arduino.h>
#include <Notecard.h>

// Forward declaration
class DataMode;

struct TimestampResult {
    unsigned long unixTime;
    bool success;
};

class CollectMode {
private:
    Notecard* notecard;
    DataMode* dataMode;
    unsigned long storedTimestamp;
    bool hasStoredTimestamp;

public:
    CollectMode();

    bool begin(Notecard* nc, DataMode* dm);
    TimestampResult getNotecardTimestamp();
    void storeTimestamp(unsigned long timestamp);
    unsigned long getStoredTimestamp();
    bool hasValidStoredTimestamp();
    void sendData();  // Will expand this to send acceleration + GPS + state data later
    void sendTimestampOnly();  // Send only timestamp data
    void sendStateLog(unsigned long utcTimestamp, unsigned long currentRTCTime);  // Send statelog format

    // Send state events using simple arrays
    void sendAllStateEvents(unsigned long* startTimes, unsigned long* endTimes, int* stateLogs, int eventCount);

private:
    void sendAccelerationData();  // For now, just acceleration data
};

#endif // COLLECT_MODE_H