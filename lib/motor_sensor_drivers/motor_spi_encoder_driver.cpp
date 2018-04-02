#include "motor_spi_encoder_driver.h"

MotorSPIEncoderDriver::MotorSPIEncoderDriver(int sensorCount, int* sensorPins)
: MotorSensorDriverABC()
{
  numSensors = (sensorCount) > 8 ? 8 : numSensors;
  for(int i=0; i < numSensors; i++)
  {
    Encoder_Buffer enc(sensorPins[i]);
    encs[i] = &enc;
    encs[i]->initEncoder();
  }
}

// update encoder information
int MotorSPIEncoderDriver::update(){
  int new_encoders[8];
  unsigned int new_timestamps[8];
  // get new sensor information
  for(int i=0; i<numSensors; i++)
  {
    new_timestamps[i] = millis();
    new_encoders[i] = encs[i]->readEncoder();
  }
  // update position and velocity information
  for(int i=0; i<numSensors; i++)
  {
    // get new position value
    angle_positions[i] = new_encoders[i] * angle_gain[i] + angle_bias[i];
    // get delta time
    double dt = ((double)new_timestamps[i] - prev_timestamps[i])/1000.0;
    // angle bias isn't needed for velocity
    angle_velocities[i] = angle_gain[i]*((double) (new_encoders[i] - prev_encoders[i]))/dt;
    // now, log timestamps as old values and record new encoder postions
    prev_timestamps[i] = new_timestamps[i];
    prev_encoders[i] = new_encoders[i];
  }
  return 0;
}
