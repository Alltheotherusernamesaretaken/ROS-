#include "motor_spi_encoder_driver.h"

MotorSPIEncoderDriver::MotorSPIEncoderDriver(int sensorCount, int* sensorPins)
: MotorSensorDriverABC()
{
  numSensors = (sensorCount) > 4 ? 4 : sensorCount;
  for(int i=0; i < numSensors; i++)
  {
    //Encoder_Buffer enc(sensorPins[i]);
    encs[i] = new Encoder_Buffer(sensorPins[i]);
  }
}

int MotorSPIEncoderDriver::begin(){
  for(int i=0; i < numSensors; i++)
  {
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

SPIRollingAverageEncoderDriver::SPIRollingAverageEncoderDriver(int sensorCount, int* sensorPins, int _rollCount)
: MotorSPIEncoderDriver(sensorCount, sensorPins)
{
  rollCount = _rollCount;
  for(int i=0; i<numSensors; i++)
  {
    roll_pos[i] = new double[rollCount];
    roll_vel[i] = new double[rollCount];
    for(int j=0; j<rollCount;j++)
    {
      roll_pos[i][j] = 0;
      roll_vel[i][j] = 0;
    }
  }
}

int SPIRollingAverageEncoderDriver::update(){
  static int rollIdx = 0;
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
    roll_pos[i][rollIdx] = new_encoders[i] * angle_gain[i] + angle_bias[i];
    // get delta time
    double dt = ((double)new_timestamps[i] - prev_timestamps[i])/1000.0;
    // angle bias isn't needed for velocity
    roll_vel[i][rollIdx] = angle_gain[i]*((double) (new_encoders[i] - prev_encoders[i]))/dt;
    
    rollIdx = (rollIdx+1)%rollCount;
    // now, log timestamps as old values and record new encoder postions
    prev_timestamps[i] = new_timestamps[i];
    prev_encoders[i] = new_encoders[i];
    double pos_sum, vel_sum;
    for (int j=0; j<rollCount ; j++)
    {
      pos_sum = pos_sum + roll_pos[i][j];
      vel_sum = vel_sum + roll_vel[i][j];
    }
    angle_positions[i] = pos_sum;
    angle_velocities[i] = vel_sum;
  }



  return 0;
}
