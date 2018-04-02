#ifndef MOTOR_SPI_ENCODER_DRIVER_ABC_H_
#define MOTOR_SPI_ENCODER_DRIVER_ABC_H_

#include<Arduino.h>
#include "motor_sensor_driver_ABC.h"
#include "../Encoder-Buffer-Library/Encoder_Buffer.h"

class MotorSPIEncoderDriver :  public MotorSensorDriverABC
{
public:

  MotorSPIEncoderDriver(int, int*);

  int update();

private:
  Encoder_Buffer* encs[8];
  int prev_encoders[8];
};

#endif
