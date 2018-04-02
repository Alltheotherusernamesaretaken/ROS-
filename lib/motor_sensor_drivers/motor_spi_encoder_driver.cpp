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
    angle_velocities[i] = angle_gain[i]*(new_encoders[i] - prev_encoders[i])/dt;
    // now, log timestamps as old values and record new encoder postions
    prev_timestamps[i] = new_timestamps[i];
    prev_encoders[i] = new_encoders[i];
  }
  return 0;
}

// get currently stored position value
int MotorSPIEncoderDriver::get_angular_position(int index, double* position){
  *position = angle_positions[index];
  return 0;
}
// get all stored positons
int MotorSPIEncoderDriver::get_angular_positions(int* sensorCount, double** positions){
  *sensorCount = numSensors;
  for (int i = 0; i<numSensors; i++)
  {
    (*positions)[i] = angle_positions[i];
  }
  return 0;
}
// get stored velocity
int MotorSPIEncoderDriver::get_angular_velocity(int index, double* velocity){
  *velocity = angle_velocities[index];
  return 0;
}
// get all stored velocities
int MotorSPIEncoderDriver::get_angular_velocities(int* sensorCount, double** velocities){
  *sensorCount = numSensors;
  for (int i = 0; i<numSensors; i++)
  {
    (*velocities)[i] = angle_velocities[i];
  }
  return 0;
}
// reset position bias to put current positon at target value
int MotorSPIEncoderDriver::reset_position(int index, double target_value){
  angle_bias[index] = angle_bias[index] - angle_positions[index] + target_value;
  angle_positions[index] = target_value;
  return 0;
}
