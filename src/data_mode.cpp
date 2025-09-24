#include "data_mode.h"
#include "LSM6DSOXSensor.h"
#include "onoff.h"

// LSM6DSOX I2C addresses
#define LSM6DSOX_ADDRESS_LOW  0x6A
#define LSM6DSOX_ADDRESS_HIGH 0x6B
#define LSM6DSOX_WHO_AM_I_VALUE 0x6C

// Components
LSM6DSOXSensor AccGyr(&Wire, LSM6DSOX_I2C_ADD_L);

// MLC variables
ucf_line_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;

// Data variables
uint8_t lsm6dsox_address = 0;
bool lsm6dsox_found = false;

DataMode::DataMode() : initialized(false), accelerometerReady(false), lastSample(0),
    isLogging(false), loggingStartTime(0), current_odr(26.0f),
    logging_duration(10000), collected_samples(0), notecard(nullptr), currentModePtr(nullptr), utcTimestamp(0), accelerometer(nullptr) {

    // Calculate sample interval from ODR
    sample_interval_ms = (unsigned long)(1000.0f / current_odr);
}

bool DataMode::begin(Notecard* nc) {

    // Store notecard reference
    notecard = nc;

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);

    if (initializeAccelerometer()) {
        initialized = true;
        accelerometerReady = true;



        // Auto-start logging like in previous example
        startLogging();
        return true;
    } else {
        return false;
    }
}

void DataMode::update() {
    if (!initialized || !accelerometerReady) {
        return;
    }

    // Handle logging (should always be active in this mode)
    if (isLogging) {
        logAccelerationData();
    }
    // Don't auto-restart - let it switch to collect mode after completion
}

bool DataMode::isAccelerometerReady() {
    return accelerometerReady;
}

bool DataMode::initializeAccelerometer() {

    // Try to initialize the sensor using the library
    if (AccGyr.begin() != LSM6DSOX_OK) {
        return false;
    }


    // Enable accelerometer
    if (AccGyr.Enable_X() != LSM6DSOX_OK) {
        return false;
    }


    // Set accelerometer configuration: 26Hz, ±2g (like in previous example)
    if (AccGyr.Set_X_ODR(26.0f) != LSM6DSOX_OK) {
        return false;
    }

    if (AccGyr.Set_X_FS(2) != LSM6DSOX_OK) {
        return false;
    }


    // Load MLC configuration for motion detection

    ProgramPointer = (ucf_line_t *)onoff;
    TotalNumberOfLine = sizeof(onoff) / sizeof(ucf_line_t);

    for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++) {
        if (AccGyr.Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
            return false;
        }
    }


    // Store accelerometer reference for MLC state reading
    accelerometer = &AccGyr;

    delay(100); // Allow sensor to stabilize

    return true;
}

void DataMode::readAndPrintAcceleration() {
    int32_t accelerometer[3];

    // Read acceleration data
    if (AccGyr.Get_X_Axes(accelerometer) == LSM6DSOX_OK) {

        // Also check MLC state
        uint8_t mlc_out[8];
        if (AccGyr.Get_MLC_Output(mlc_out) == LSM6DSOX_OK) {
        }
    } else {
    }
}

void DataMode::startLogging() {

    isLogging = true;
    loggingStartTime = millis();
    collected_samples = 0;
    lastSample = 0;

    digitalWrite(LED_BUILTIN, HIGH);
}

void DataMode::stopLogging() {
    isLogging = false;
    digitalWrite(LED_BUILTIN, LOW);


    // Send all samples to cloud
    sendSamplesToCloud();

    // Auto-switch to COLLECT MODE (mode 0)
    if (currentModePtr != nullptr) {
        *currentModePtr = 0;
        delay(1000); // Brief pause for mode switch
    }
}

bool DataMode::getIsLogging() {
    return isLogging;
}

void DataMode::logAccelerationData() {
    // Check if logging duration exceeded
    if (millis() - loggingStartTime >= logging_duration) {
        stopLogging();
        return;
    }

    // Check if we've collected maximum samples
    if (collected_samples >= MAX_SAMPLES) {
        stopLogging();
        return;
    }

    // Sample at the specified interval
    if (millis() - lastSample >= sample_interval_ms) {
        int32_t accelerometer[3];

        if (AccGyr.Get_X_Axes(accelerometer) == LSM6DSOX_OK) {
            // Convert to float and store in arrays (mg values)
            ax_samples[collected_samples] = (float)accelerometer[0];
            ay_samples[collected_samples] = (float)accelerometer[1];
            az_samples[collected_samples] = (float)accelerometer[2];


            collected_samples++;
        }

        lastSample = millis();
    }
}

void DataMode::sendSamplesToCloud() {
    if (collected_samples == 0) {
        return;
    }

    if (notecard == nullptr) {
        return;
    }

    writeBinaryData();
}

void DataMode::writeBinaryData() {
    // Send acceleration data as base64-encoded JSON note (same as previous example)

    // Calculate total size needed
    int total_size = collected_samples * 12;  // 3 floats * 4 bytes each

    // Create buffer with all data
    uint8_t* all_data = (uint8_t*)malloc(total_size);
    if (all_data == NULL) {
        return;
    }

    // Pack all samples into the buffer
    for (int i = 0; i < collected_samples; i++) {
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
        JAddNumberToObject(body, "samples", collected_samples);
        JAddNumberToObject(body, "format", 1);  // 1 = float32 ax,ay,az format
        JAddNumberToObject(body, "rate_hz", current_odr);
        JAddNumberToObject(body, "duration_ms", logging_duration);
        JAddNumberToObject(body, "timestamp", utcTimestamp); // Using UTC timestamp
    }

    bool success = notecard->sendRequest(req);

    if (success) {
    } else {
    }

    // Clean up
    free(all_data);
    free(encoded);
}

void DataMode::setModePointer(int* modePtr) {
    currentModePtr = modePtr;
}

void DataMode::setUTCTimestamp(unsigned long timestamp) {
    utcTimestamp = timestamp;
}

float* DataMode::getAxSamples() {
    return ax_samples;
}

float* DataMode::getAySamples() {
    return ay_samples;
}

float* DataMode::getAzSamples() {
    return az_samples;
}

int DataMode::getCollectedSamples() {
    return collected_samples;
}

float DataMode::getCurrentODR() {
    return current_odr;
}

unsigned long DataMode::getLoggingDuration() {
    return logging_duration;
}

uint8_t DataMode::getCurrentMlcState() {
    if (accelerometer == nullptr) {
        return 0;
    }

    uint8_t mlc_out[8];
    if (accelerometer->Get_MLC_Output(mlc_out) == LSM6DSOX_OK) {
        return mlc_out[0]; // Return first MLC output
    }
    return 0;
}