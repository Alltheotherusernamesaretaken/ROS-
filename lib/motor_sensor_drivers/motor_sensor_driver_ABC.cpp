#include "motor_sensor_driver_ABC.h"

MotorSensorDriverABC::MotorSensorDriverABC(int sensorCount)
{
  numSensors = (sensorCount) > 8 ? 8 : numSensors;
}
