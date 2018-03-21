#include "motor_sensor_driver_abc.h"

MotorSensorDriverABC::MotorSensorDriverABC(int sensorCount)
{
  numSensors = (sensorCount) > 8 ? 8 : numSensors;
}
