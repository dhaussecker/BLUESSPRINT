#ifndef DATA_MODE_H
#define DATA_MODE_H

#include <Arduino.h>
#include <Wire.h>
#include <Notecard.h>

// Data storage for batching (same as previous example)
#define MAX_SAMPLES 300

class DataMode {
private:
    bool initialized;
    bool accelerometerReady;
    unsigned long lastSample;
    bool isLogging;
    unsigned long loggingStartTime;

    // Configuration (same as previous example)
    float current_odr;
    unsigned long sample_interval_ms;
    unsigned long logging_duration;

    // Data storage arrays
    float ax_samples[MAX_SAMPLES];
    float ay_samples[MAX_SAMPLES];
    float az_samples[MAX_SAMPLES];
    int collected_samples;

    // External notecard reference
    Notecard* notecard;

    // Pointer to global currentMode variable
    int* currentModePtr;

public:
    DataMode();

    bool begin(Notecard* nc = nullptr);
    void update();
    bool isAccelerometerReady();
    void startLogging();
    void stopLogging();
    bool getIsLogging();
    void setModePointer(int* modePtr);

private:
    bool initializeAccelerometer();
    void readAndPrintAcceleration();
    void logAccelerationData();
    void sendSamplesToCloud();
    void writeBinaryData();
};

#endif // DATA_MODE_H