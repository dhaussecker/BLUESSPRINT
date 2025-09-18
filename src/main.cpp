#include <Arduino.h>
#include <STM32LowPower.h>
#include <Notecard.h>

Notecard notecard;

void setup() {
  // Configure LED pin
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize the low power library
  LowPower.begin();
  notecard.begin();

  {
    J *req = notecard.newRequest("hub.set");
    if (req != NULL) {
      JAddStringToObject(req, "mode", "periodic");
      JAddNumberToObject(req, "outbound", 5);
      JAddNumberToObject(req, "inbound", 5);
      notecard.sendRequest(req);
      Serial.println("Notecard forced into deep sleep");
    }
  }
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  LowPower.deepSleep(10000); // Deep sleep for 1 second
  digitalWrite(LED_BUILTIN, LOW);
  LowPower.deepSleep(10000);
}