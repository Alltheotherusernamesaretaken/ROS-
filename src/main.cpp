#include"main.h"


void setup() {
  // Serial Debug output
  Serial.begin(115200);
  Serial.println("Booting");

  // Start wifi
  start_WiFi();
  // Start OTA server
  start_OTA();

  // Spin up WiFi/etc. loop on Core 0
  xTaskCreatePinnedToCore(
    WiFi_loop, // Task function
    "WiFi_loop", // Task name
    10000, // Stack size for task (in words)
    NULL,  // Parameter passed as input
    2,     // Task priority
    NULL,  // Task handle
    0 // Run on Core 0
  );
}

// WiFi and high level async task loop
// This loop shares Core 0 with the WiFi handling and contains all processes
//    which are not time-critical, leaving Core 1 for synchronous tasks
//    with strict timing requirements.
void WiFi_loop(void * parameter) {
  while (1){
    static long start;
    start = millis();
    // Handles OTA updates
    ArduinoOTA.handle();

    yield(); // yield to let the WiFi drivers do their thing

    // Sleep for remainder to give 100 Hz
    if ((millis()-start)<10) vTaskDelay((10-(millis()-start))/portTICK_PERIOD_MS);
  }
}

// Main loop
// The ESP32 has two cores (0 and 1). WiFi and other non-timing-critical
//    tasks should be done on core 0. Core 1 is dedicated to PID and timing
//    sensitive tasks.
// The Arduino loop always runs on Core 1
void loop() {
  static uint32_t start;
  start = millis();

  // Sleep for remainder to give 100 Hz
  if ((millis()-start)<10) vTaskDelay((10-(millis()-start))/portTICK_PERIOD_MS);
}
