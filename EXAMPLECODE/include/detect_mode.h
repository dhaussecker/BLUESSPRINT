#ifndef DETECT_MODE_H
#define DETECT_MODE_H

#include <Notecard.h>
#include <STM32RTC.h>
#include <STM32LowPower.h>
#include "gps_tracker.h"
#include "LSM6DSOXSensor.h"

#define MAX_STATE_EVENTS 50

struct AccelStateEvent {
    uint8_t mlcState;           // MLC motion classification (0-255)
    unsigned long startTime;    // Unix timestamp when event started
    unsigned long endTime;      // Unix timestamp when event ended
    String eventType;           // "timestamp_sync", "timer_wake", "interrupt_wake", "final_capture"
};

enum DetectStage {
    STAGE_TIMESTAMP_COLLECTION = 1,
    STAGE_2 = 2
};

class DetectMode {
private:
    DetectStage currentStage;
    Notecard& notecard;
    STM32RTC& rtc;
    bool timestampCollected;
    unsigned long stageStartTime;
    GPSTracker gpsTracker;

    // Accelerometer state storage
    AccelStateEvent stateEvents[MAX_STATE_EVENTS];
    int eventCount;
    unsigned long lastStateTime;
    uint8_t lastMlcState;
    LSM6DSOXSensor* accelerometer;
    bool accelInitialized;

    // Transmission timing
    unsigned long lastTransmissionTime;
    const unsigned long TRANSMISSION_INTERVAL = 3600; // Send every hour (3600 seconds)

public:
    DetectMode(Notecard& nc, STM32RTC& rtcInstance);

    void begin();
    void update();
    DetectStage getCurrentStage();
    void handleWakeInterrupt();

private:
    void handleStage1();
    void handleStage2();

    // Accelerometer methods
    bool initializeAccelerometer();
    uint8_t readMlcState();
    void storeStateEvent(const String& eventType, unsigned long startTime, unsigned long endTime = 0);
    void sendStateEventsToCloud();
    bool shouldSendData();
    void addFinalStateAndSend();
};

#endif // DETECT_MODE_H