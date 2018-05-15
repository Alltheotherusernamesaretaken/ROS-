#ifndef MAIN_ESP32_ROBOT_DRIVER_H_
#define MAIN_ESP32_ROBOT_DRIVER_H_

// custom library includes
#include "motor_spi_encoder_driver.h"
#include "motor_ADS1015_driver.h"
#include "PID_Controller.h"
#include "UDP_interfaces.h"

#include <Arduino.h>
// WiFi and OTA libraries
#include <WiFi.h> // ESP32 WiFi drivers
#include <ESPmDNS.h> // ESP32 mDNS support
#include <WiFiUdp.h> // ESP32 WiFi UDP
#include <ArduinoOTA.h> // Arduino OTA updates
#include <Update.h> // Needed for OTA
#include <Wire.h>
// Identify which core has the Arduino stuff
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif


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
void WiFi_loop(void * parameter);



#endif
