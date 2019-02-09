#ifndef MAIN_ESP32_ROBOT_DRIVER_H_
#define MAIN_ESP32_ROBOT_DRIVER_H_

#include <Arduino.h>
// WiFi and OTA libraries
#include <WiFi.h> // ESP32 WiFi drivers
#include <ESPmDNS.h> // ESP32 mDNS support
#include <WiFiUdp.h> // ESP32 WiFi UDP
#include <ArduinoOTA.h> // Arduino OTA updates
#include <Update.h> // Needed for OTA

// Identify which core has the Arduino stuff
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif


void WiFi_loop(void * parameter);

// Set ssid and password here
const char* ssid = "Team_13";
const char* password = "StateSpaceRobotics";
extern const char* hostname;

// Inlined to encourage compiler optimization
inline void start_WiFi() {
  // Start up wifi and connect
  Serial.printf("Hostname: %s\n", hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setHostname(hostname);

  // Restart until successful connect
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Print the IP
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void inline start_OTA() {
  // Port for OTA server
  ArduinoOTA.setPort(3232);

  // Hostname for mDNS resolving
  ArduinoOTA.setHostname(hostname);

  // Password for performing OTA update
  ArduinoOTA.setPassword("OTApass");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  // Setup handler functions
  ArduinoOTA
    // Runs this function on start
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    // Runs this function on end
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    // Runs this function during update
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    // Runs this if there's an error
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  // Starts the OTA server
  ArduinoOTA.begin();

}
void setup() {
  // Serial Debug output
  Serial.begin(115200);
  Serial.println("Booting");

  delay(2000);

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


#endif
