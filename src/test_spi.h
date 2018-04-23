#ifndef TEST_SPI
#define TEST_SPI
#define TEST
// custom library includes
#include "motor_spi_encoder_driver.h"
// Configs
int numberOfEncoders = 1;
int encoderSelectPinArray[] = {15};

// Globals
MotorSPIEncoderDriver SPI_enc_interface(numberOfEncoders, encoderSelectPinArray);

#endif
