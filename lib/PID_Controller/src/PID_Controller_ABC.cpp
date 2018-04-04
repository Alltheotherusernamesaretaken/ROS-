#include "PID_Controller_ABC.h"

PIDControllerABC::PIDControllerABC(uint8_t controlType, uint8_t proportionType, int PWMChannelOffset, int _numPID, int* PIDArray)
{
  // Immediately setup and lock
  mutex = xSemaphoreCreateMutex();
  lock();
  numPID = _numPID;
  motControlType = controlType;
  motProportionType = proportionType;

  for (int i=0; i < numPID; i++)
  {
    motPWMPin[i] = PIDArray[i];
    motPWMChannel[i] = PWMChannelOffset+i;

    // Initialize PWM channels
    ledcSetup(motPWMChannel[i], 20000, 8);
    ledcAttachPin(motPWMPin[i], motPWMChannel[i]);

    // Initialize PID
    motPID[i] = new PID(&motSensorVal[0], &motPWM[0], &motSetpoint[0], 0, 0, 0, ((motProportionType & (1<<i))>>i), DIRECT);
    motPID[i]->SetOutputLimits(-255,255);

    // zero the setpoint, to be safe
    motSetpoint[i] = 0;
  }
  unlock();
}

void inline PIDControllerABC::lock()
{
  xSemaphoreTake( mutex, portMAX_DELAY );
}

void inline PIDControllerABC::unlock()
{
  xSemaphoreGive( mutex );
}

virtual int PIDControllerABC::update(){
  lock();
  for (i=0, i<8, i++){

  }
  unlock();
}

int PIDControllerABC::set_PID_gains(int i, double kp, double ki, double kd){
  if (i >= numPID) return 1;
  lock();
  motPID[i]->SetTunings(kp, ki, kd);
  unlock();
  return 0;
}

int PIDControllerABC::get_PID_gains(int i, double *kp, double *ki, double *kd){
  lock();
  *kp = motPID[i]->GetKp();
  *ki = motPID[i]->GetKi();
  *kd = motPID[i]->GetKd();
  unlock();
  return 0;
}

int PIDControllerABC::set_PID_setpoint(int i,double setp){
  lock();
  motSetpoint[i] = setp;
  unlock();
  return 0;
}

int PIDControllerABC::get_PID_setpoint(int i,double *setp){
  lock();
  *setp = motSetpoint[i];
  unlock();
  return 0;
}

int PIDControllerABC::set_sensor_gain(int i,double gain){
  lock();
  sensor_driver->set_angular_gain(motSensorChannel[i],gain);
  unlock();
  return 0;
}

int PIDControllerABC::get_sensor_gain(int i,double *gain){
  lock();
  sensor_driver->get_angular_gain(motSensorChannel[i],gain);
  unlock();
  return 0;
}

int PIDControllerABC::set_sensor_bias(int i,double bias){
  lock();
  sensor_driver->set_angular_bias(motSensorChannel[i],bias);
  unlock();
  return 0;
}

int PIDControllerABC::get_sensor_bias(int i,double *bias){
  lock();
  sensor_driver->get_angular_bias(motSensorChannel[i],bias);
  unlock();
  return 0;
}

int PIDControllerABC::set_PID_control_type(uint8_t ctype){
  lock();
  motControlType = ctype;
  // TODO: need to handle when changing PID on-line
  unlock();
  return 0;
}

int PIDControllerABC::get_PID_control_type(uint8_t *ctype){
  lock();
  *ctype = motControlType;
  unlock();
  return 0;
}

int PIDControllerABC::set_PID_proportion_type(uint8_t ptype){
  lock();
  motProportionType = ptype;
  // TODO: Need to update PID
  // TODO: Need to handle when changing PID on-line
  unlock();
  return 0;
}

int PIDControllerABC::get_PID_proportion_type(uint8_t *ptype){
  lock();
  *ptype = motProportionType;
  unlock();
  return 0;
}

int PIDControllerABC::zero_PID_sensor(int i, double value){
  lock();
  // TODO: zero underlying object
  unlock();
  return 0;
}

int PIDControllerABC::_write_PWM_values(){
  lock();
  for(int i=0; i<numPID; i++){
    // TODO: write pwm
  }
  unlock();
  return 0;
}
