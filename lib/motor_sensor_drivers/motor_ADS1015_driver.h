#ifndef MOTOR_ADS1015_DRIVER_ABC_H_
#define MOTOR_ADS1015_DRIVER_ABC_H_

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include "motor_sensor_driver_ABC.h"

class MotorADS1015Driver :  public MotorSensorDriverABC
{
public:

  MotorADS1015Driver(int, int*);

  int update();

  int get_angular_position(int, double*);
  int get_angular_positions(int*, double**);

  int get_angular_velocity(int, double*);
  int get_angular_velocities(int*, double**);

  int reset_position(int, double);

  int get_angular_gain(int, double*);
  int set_angular_gain(int, double);

  int get_angular_bias(int, double*);
  int set_angular_bias(int, double);

private:
  Adafruit_ADS1015 ads;
  int16_t prev_adcs[4];
};

#endif
