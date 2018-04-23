#include"main.h"

#include <SPI.h>

// testing includes
//#include "test_adc.h"
//#include "test_spi.h"
#include "test_pid_spi.h"
//#include "test_pid_adc.h"

//#define TASK_DEBUG

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

  #ifdef TEST_SPI
    SPI.begin(14,27,13,15);
    SPI_enc_interface.begin();
    SPI_enc_interface.set_angular_gain(0,1);
    #ifdef TEST_PID_SPI
      spi_pid.set_sensor_gain(0,1);
      spi_pid.set_PID_gains(0, 0, 0.1, 0);
      spi_pid.set_PID_setpoint(0, 1000);
      double set;
      spi_pid.get_PID_setpoint(0,&set);
      Serial.println(set);
    #endif
  #endif
  #ifdef TEST_ADC
    Wire.begin(21,22);
    adc_interface.set_angular_gain(0,1);
    #ifdef TEST_PID_ADC
      adc_pid.set_sensor_gain(0,1);
      adc_pid.set_PID_gains(0, 1, 1, 1);
      adc_pid.set_PID_setpoint(0, 20);
    #endif
  #endif

  // Actual Running code
  #ifndef TEST
    init_enc();
    init_PID();
  #endif
}

// WiFi and high level async task loop
// This loop shares Core 0 with the WiFi handling and contains all processes
//    which are not time-critical, leaving Core 1 for synchronous tasks
//    with strict timing requirements.
void WiFi_loop(void * parameter) {
  while (1){
    // Handles OTA updates
    ArduinoOTA.handle();
    #ifdef TASK_DEBUG
      Serial.print("WiFi_loop: ");
      Serial.println(xPortGetCoreID());
    #endif

    yield(); // yield to let the WiFi drivers do their thing
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
}

// Main loop
// The ESP32 has two cores (0 and 1). WiFi and other non-timing-critical
//    tasks should be done on core 0. Core 1 is dedicated to PID and timing
//    sensitive tasks.
// The Arduino loop always runs on Core 1
void loop() {
  #ifdef TASK_DEBUG
    Serial.print("main_loop: ");
    Serial.println(xPortGetCoreID());
  #endif

  #ifdef TEST_SPI
    #ifdef TEST_PID_SPI
      Serial.print("PID Encoder update: ");
      Serial.println(spi_pid.update());
      double val;
      spi_pid.get_PID_setpoint(0, &val);
      Serial.println(val);
      spi_pid.get_PID_output(0, &val);
      Serial.println(val);
    #else
      Serial.print("Encoder update Error: ");
      Serial.println(SPI_enc_interface.update());
    #endif
    double enc;
    Serial.print("Encoder get_angular_positon Error: ");
    Serial.println(SPI_enc_interface.get_angular_position(0,&enc));
    Serial.print("Encoder: "); Serial.println(enc);
    Serial.print("Encoder get_angular_velocity Error: ");
    Serial.println(SPI_enc_interface.get_angular_velocity(0,&enc));
    Serial.print("Encoder: "); Serial.println(enc);
  #endif
  #ifdef TEST_ADC
    #ifdef TEST_PID_ADC
      Serial.print("PID ADC update: ");
      Serial.println(adc_pid.update());
    #else
      Serial.print("ADC update Error: ");
      Serial.println(adc_interface.update());
    #endif
    double adc;
    Serial.print("ADC get_angular_position Error: ");
    Serial.println(adc_interface.get_angular_position(0,&adc));
    Serial.print("ADC: "); Serial.println(adc);
    Serial.print("ADC get_angular_velocity Error: ");
    Serial.println(adc_interface.get_angular_velocity(0,&adc));
    Serial.print("ADC: "); Serial.println(adc);
  #endif
  vTaskDelay(100/portTICK_PERIOD_MS);

}
