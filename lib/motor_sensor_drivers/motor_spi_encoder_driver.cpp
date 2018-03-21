#include "motor_spi_encoder_driver.h"

MotorSPIEncoderDriver::MotorSPIEncoderDriver(int sensorCount, int* sensorPins)
: MotorSensorDriverABC(sensorCount)
{
  for(int i=0; i < numSensors; i++)
  {
    Encoder_Buffer enc(sensorPins[i]);
    encs[i] = &enc;
    encs[i]->initEncoder();
  }
}

int MotorSPIEncoderDriver::read_sensor(int sensor, int32_t *val)
{
  *val = encs[sensor]->readEncoder();
}

int MotorSPIEncoderDriver::read_sensors(int* sensorCount, int32_t **vals)
{
  *sensorCount = numSensors;
  for(int i=0; i<numSensors; i++)
  {
    (*vals)[i] = encs[i]->readEncoder();
  }
}
