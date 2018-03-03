#ifndef PID_FUNCTIONS_H
#define PID_FUNCTIONS_H
#include <PID_v1.h>
#include <Arduino.h>

// Global variables
// Generally poor practice, but ok in embedded systems
// PID variables
extern const int EN1, EN2;
extern const int PWM_PIN;
extern const int freq; // max frequency is 80,000,000 / 2^resolution
extern const int mot_channel;
extern const int resolution;
extern double cmd_vel, velocity, pwm;
extern double Ki, Kp, Kd;
extern PID PID_ex;

// PID function declarations
void setup_PID();
void do_PID();

#endif
