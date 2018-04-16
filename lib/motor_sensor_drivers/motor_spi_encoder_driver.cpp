#include "motor_spi_encoder_driver.h"

MotorSPIEncoderDriver::MotorSPIEncoderDriver(int sensorCount, int* sensorPins)
: MotorSensorDriverABC()
{
  numSensors = (sensorCount) > 4 ? 4 : sensorCount;
  for(int i=0; i < numSensors; i++)
  {
    //Encoder_Buffer enc(sensorPins[i]);
    encs[i] = new Encoder_Buffer(sensorPins[i]);
    encs[i]->initEncoder();
  }
}

// update encoder information
int MotorSPIEncoderDriver::update(){
  int new_encoders[4];
  unsigned int new_timestamps[4];
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
