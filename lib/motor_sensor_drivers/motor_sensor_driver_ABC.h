#ifndef MOTOR_SENSOR_DRIVER_ABC_H_
#define MOTOR_SENSOR_DRIVER_ABC_H_

#include<Arduino.h>

class MotorSensorDriverABC
{
public:

  MotorSensorDriverABC(){};

  virtual int update();

  int get_angular_position(int index, double* position){
    if (index >= numSensors) return 1;
    *position = angle_positions[index];
    return 0;
  }

  int get_angular_positions(int* sensorCount, double** positions){
    *sensorCount = numSensors;
    for (int i = 0; i<numSensors; i++)
    {
      (*positions)[i] = angle_positions[i];
    }
    return 0;
  }

  int get_angular_velocity(int index, double* velocity){
    if (index >= numSensors) return 1;
    *velocity = angle_velocities[index];
    return 0;
  }
  int get_angular_velocities(int* sensorCount, double** velocities){
    *sensorCount = numSensors;
    for (int i = 0; i<numSensors; i++)
    {
      (*velocities)[i] = angle_velocities[i];
    }
    return 0;
  }

  int reset_position(int index, double target_value){
    if (index >= numSensors) return 1;
    angle_bias[index] = angle_bias[index] - angle_positions[index] + target_value;
    angle_positions[index] = target_value;
    return 0;
  }

  int get_angular_gain(int index, double* gain){
    if (index >= numSensors) return 1;
    *gain = angle_gain[index];
    return 0;
  }
  int set_angular_gain(int index, double gain){
    if (index >= numSensors) return 1;
    angle_gain[index] = gain;
    return 0;
  }

  int get_angular_bias(int index, double* bias){
    if (index >= numSensors) return 1;
    *bias = angle_bias[index];
    return 0;
  }
  int set_angular_bias(int index, double bias){
    if (index >= numSensors) return 1;
    angle_bias[index] = bias;
    return 0;
  }


protected:
  int numSensors = 0;

  unsigned int prev_timestamps[4] = {0};

  double angle_positions[4] = {0};
  double angle_velocities[4] = {0};

  double angle_bias[4] = {0};
  double angle_gain[4] = {0};
};

#endif
