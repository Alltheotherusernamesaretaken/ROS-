#ifndef TEST_PID_SPI
#define TEST_PID_SPI
#define TEST

// Enable spi test
#include "test_spi.h"

// custom library includes
#include "PID_Controller.h"
// Configs
uint8_t SPIcontrolType = 0b00000000;
uint8_t SPIproportionType = 0b00000000;
int SPIPWMChannelOffset = 0;
int SPInumPID = 1;
int SPIPWMpins[1] = {2};
int SPIPIDSensorChannels[1] = {0};

PIDController spi_pid(
  SPIcontrolType,
  SPIproportionType,
  SPIPWMChannelOffset,
  SPInumPID,
  SPIPWMpins,
  SPIPIDSensorChannels,
  &SPI_enc_interface
);

#endif
