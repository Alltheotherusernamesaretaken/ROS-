#ifndef MOTOR_SENSOR_DRIVER_ABC_H_
#define MOTOR_SENSOR_DRIVER_ABC_H_

#include<Arduino.h>

class MotorSensorDriverABC
{
public:

  MotorSensorDriverABC(int);

  virtual int update();

  virtual int get_angular_position(int, double*);
  virtual int get_angular_positions(int*, double**);

  virtual int get_angular_velocity(int, double*);
  virtual int get_angular_velocities(int*, double**);

  int reset_position(int, double);

  virtual int get_angular_gain(int, double*);
  virtual int set_angular_gain(int, double);

  virtual int get_angular_bias(int, double*);
  virtual int set_angular_bias(int, double);


protected:
  int numSensors;

  unsigned int prev_timestamps[8];

  double angle_positions[8];
  double angle_velocities[8];

  double angle_bias[8];
  double angle_gain[8];
};

#endif
