#ifndef MOTOR_SENSOR_DRIVER_ABC_H_
#define MOTOR_SENSOR_DRIVER_ABC_H_

#include<Arduino.h>

class MotorSensorDriverABC
{
public:

  MotorSensorDriverABC(int);

  virtual int read_sensors(int*, int32_t**);

  virtual int read_sensor(int, int32_t*);

protected:
  int numSensors;
};

#endif
