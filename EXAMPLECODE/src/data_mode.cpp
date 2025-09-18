#include "data_mode.h"
#include "LSM6DSOXSensor.h"
#include "movement.h"

// LSM6DSOX I2C addresses
#define LSM6DSOX_ADDRESS_LOW  0x6A
#define LSM6DSOX_ADDRESS_HIGH 0x6B
#define LSM6DSOX_WHO_AM_I_VALUE 0x6C

// LSM6DSOX register addresses (from previous implementation)
#define LSM6DSOX_WHO_AM_I     0x0F
#define LSM6DSOX_CTRL1_XL     0x10
#define LSM6DSOX_STATUS_REG   0x1E
#define LSM6DSOX_OUTX_L_A     0x28

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
    logging_duration(10000), collected_samples(0), notecard(nullptr), currentModePtr(nullptr) {

    // Calculate sample interval from ODR
    sample_interval_ms = (unsigned long)(1000.0f / current_odr);
}

bool DataMode::begin(Notecard* nc) {
    Serial.println("=== DATA MODE INITIALIZING ===");

    // Store notecard reference
    notecard = nc;

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);
    Serial.println("I2C initialized at 400kHz");

    if (initializeAccelerometer()) {
        initialized = true;
        accelerometerReady = true;

        Serial.print("Max samples per session: ");
        Serial.println(MAX_SAMPLES);
        Serial.print("Sample interval: ");
        Serial.print(sample_interval_ms);
        Serial.println(" ms");
        Serial.print("Logging duration: ");
        Serial.print(logging_duration / 1000);
        Serial.println(" seconds");

        Serial.println("=== DATA MODE READY ===");
        Serial.println("Starting logging automatically...");

        // Auto-start logging like in previous example
        startLogging();
        return true;
    } else {
        Serial.println("=== DATA MODE FAILED TO INITIALIZE ===");
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
    Serial.println("Initializing LSM6DSOX accelerometer...");

    // Try to initialize the sensor using the library
    if (AccGyr.begin() != LSM6DSOX_OK) {
        Serial.println("Failed to initialize LSM6DSOX library");
        return false;
    }

    Serial.println("LSM6DSOX library initialized successfully");

    // Enable accelerometer
    if (AccGyr.Enable_X() != LSM6DSOX_OK) {
        Serial.println("Failed to enable accelerometer");
        return false;
    }

    Serial.println("Accelerometer enabled");

    // Set accelerometer configuration: 26Hz, ±2g (like in previous example)
    if (AccGyr.Set_X_ODR(26.0f) != LSM6DSOX_OK) {
        Serial.println("Failed to set accelerometer ODR");
        return false;
    }

    if (AccGyr.Set_X_FS(2) != LSM6DSOX_OK) {
        Serial.println("Failed to set accelerometer full scale");
        return false;
    }

    Serial.println("Accelerometer configured: 26Hz, ±2g");

    // Load MLC configuration for motion detection
    Serial.println("Loading MLC configuration...");

    ProgramPointer = (ucf_line_t *)movement;
    TotalNumberOfLine = sizeof(movement) / sizeof(ucf_line_t);
    Serial.print("UCF Number of Lines: ");
    Serial.println(TotalNumberOfLine);

    for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++) {
        if (AccGyr.Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
            Serial.print("Error loading MLC program at line: ");
            Serial.println(LineCounter);
            return false;
        }
    }

    Serial.println("MLC program loaded successfully");

    delay(100); // Allow sensor to stabilize

    return true;
}

void DataMode::readAndPrintAcceleration() {
    int32_t accelerometer[3];

    // Read acceleration data
    if (AccGyr.Get_X_Axes(accelerometer) == LSM6DSOX_OK) {
        Serial.print("Acceleration [mg]: X=");
        Serial.print(accelerometer[0]);
        Serial.print(", Y=");
        Serial.print(accelerometer[1]);
        Serial.print(", Z=");
        Serial.println(accelerometer[2]);

        // Also check MLC state
        uint8_t mlc_out[8];
        if (AccGyr.Get_MLC_Output(mlc_out) == LSM6DSOX_OK) {
            Serial.print("MLC State: ");
            Serial.println(mlc_out[0]);
        }
    } else {
        Serial.println("Failed to read acceleration data");
    }
}

void DataMode::startLogging() {
    Serial.println("=== STARTING DATA LOGGING SESSION ===");
    Serial.println("A_X [mg]\tA_Y [mg]\tA_Z [mg]");
    Serial.print("Logging for ");
    Serial.print(logging_duration / 1000);
    Serial.println(" seconds...");

    isLogging = true;
    loggingStartTime = millis();
    collected_samples = 0;
    lastSample = 0;

    digitalWrite(LED_BUILTIN, HIGH);
}

void DataMode::stopLogging() {
    isLogging = false;
    digitalWrite(LED_BUILTIN, LOW);

    Serial.println("=== LOGGING COMPLETED ===");
    Serial.print("Total samples collected: ");
    Serial.println(collected_samples);
    Serial.print("Actual rate: ");
    Serial.print((float)collected_samples * 1000.0 / logging_duration, 2);
    Serial.println(" Hz");

    // Send all samples to cloud
    sendSamplesToCloud();

    // Auto-switch to COLLECT MODE (mode 0)
    if (currentModePtr != nullptr) {
        Serial.println("Auto-switching to COLLECT MODE...");
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
        Serial.println("Maximum samples reached!");
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

            // Print to serial for monitoring
            Serial.print(ax_samples[collected_samples], 1);
            Serial.print("\t");
            Serial.print(ay_samples[collected_samples], 1);
            Serial.print("\t");
            Serial.println(az_samples[collected_samples], 1);

            collected_samples++;
        }

        lastSample = millis();
    }
}

void DataMode::sendSamplesToCloud() {
    if (collected_samples == 0) {
        Serial.println("No samples to send");
        return;
    }

    if (notecard == nullptr) {
        Serial.println("Notecard not available - cannot send to cloud");
        return;
    }

    Serial.println("Sending samples to cloud as JSON note...");
    writeBinaryData();
}

void DataMode::writeBinaryData() {
    // Send acceleration data as base64-encoded JSON note (same as previous example)
    Serial.println("Encoding acceleration data as base64...");

    // Calculate total size needed
    int total_size = collected_samples * 12;  // 3 floats * 4 bytes each

    // Create buffer with all data
    uint8_t* all_data = (uint8_t*)malloc(total_size);
    if (all_data == NULL) {
        Serial.println("Failed to allocate memory for data");
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
        Serial.println("Failed to allocate memory for encoded data");
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
        JAddNumberToObject(body, "timestamp", millis()); // Using millis as timestamp
    }

    bool success = notecard->sendRequest(req);

    if (success) {
        Serial.print("Successfully sent ");
        Serial.print(collected_samples);
        Serial.println(" samples as base64 JSON note");
    } else {
        Serial.println("Failed to send data note");
    }

    // Clean up
    free(all_data);
    free(encoded);
}

void DataMode::setModePointer(int* modePtr) {
    currentModePtr = modePtr;
}