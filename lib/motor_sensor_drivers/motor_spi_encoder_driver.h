#ifndef MOTOR_SPI_ENCODER_DRIVER_ABC_H_
#define MOTOR_SPI_ENCODER_DRIVER_ABC_H_

#include<Arduino.h>
#include "motor_sensor_driver_ABC.h"
#include "../Encoder-Buffer-Library/Encoder_Buffer.h"

class MotorSPIEncoderDriver :  public MotorSensorDriverABC
{
public:

  MotorSPIEncoderDriver(int, int*);
  int begin();
  int update();

protected:
  Encoder_Buffer* encs[4];
  int prev_encoders[4];
};

class SPIRollingAverageEncoderDriver : public MotorSPIEncoderDriver
{
public:

  SPIRollingAverageEncoderDriver(int, int*, int=5);

  int update();

protected:
  int rollCount;
  double* roll_pos[4];
  double* roll_vel[4];
};

#endif
