#ifndef MOTOR_SPI_ENCODER_DRIVER_ABC_H_
#define MOTOR_SPI_ENCODER_DRIVER_ABC_H_

#include<Arduino.h>
#include "motor_sensor_driver_abc.h"
#include "../Encoder-Buffer-Library/Encoder_Buffer.h"

class MotorSPIEncoderDriver :  public MotorSensorDriverABC
{
public:

  MotorSPIEncoderDriver(int, int*);

  int read_sensors(int*, int32_t**);

  int read_sensor(int, int32_t*);

private:
  Encoder_Buffer* encs[8];
};

#endif
