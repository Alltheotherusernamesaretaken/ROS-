#include "pid_functions.h"

#include "encoder_velocity_functions.h"
// Global variables
// Generally poor practice, but ok in embedded systems
// PID variables
const int EN1 = 18, EN2 = 5;
const int PWM_PIN = 19;
const int freq = 20000; // max frequency is 80,000,000 / 2^resolution
const int mot_channel = 0;
const int resolution = 8;
double cmd_vel=0, velocity=0, pwm=0;
double Ki=3, Kp=3, Kd=0;
PID PID_ex(&velocity, &pwm, &cmd_vel, Kp, Ki, Kd, DIRECT);

void setup_PID() {
  // Direction pins
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  // Initialize PWM Pin
  ledcSetup(mot_channel, freq, resolution);
  ledcAttachPin(PWM_PIN, mot_channel);

  PID_ex.SetMode(AUTOMATIC);
  PID_ex.SetOutputLimits(-255, 255);
  cmd_vel = 0;
}


void do_PID(){
  update_vel();

  PID_ex.Compute();

  // Forward or reverse
  if (pwm > 0){
    // Forward
    // Write to direction pins
    digitalWrite(EN1, HIGH);
    digitalWrite(EN2, LOW);
  }else {
    // Reverse
    // Write to direction pins
    digitalWrite(EN1, LOW);
    digitalWrite(EN2, HIGH);

  }
  // Write PWM
  ledcWrite(mot_channel, (uint8_t) round(abs(pwm)));

}
