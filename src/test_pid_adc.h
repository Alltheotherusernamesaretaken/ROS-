#ifndef TEST_PID_ADC
#define TEST_PID_ADC
#define TEST

// Enable spi test
#include "test_adc.h"

// custom library includes
#include "PID_Controller.h"
// Configs
uint8_t ADCcontrolType = 0b00000000;
uint8_t ADCproportionType = 0b00000000;
int ADCPWMChannelOffset = 0;
int ADCnumPID = 1;
int ADCPWMpins[1] = {25};
int ADCPIDSensorChannels[1] = {0};

PIDController adc_pid(
  ADCcontrolType,
  ADCproportionType,
  ADCPWMChannelOffset,
  ADCnumPID,
  ADCPWMpins,
  ADCPIDSensorChannels,
  &adc_interface
);

#endif
