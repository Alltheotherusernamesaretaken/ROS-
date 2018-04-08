
#include "PID_Controller.h"

PIDController::PIDController(uint8_t controlType, uint8_t proportionType, int PWMChannelOffset, int _numPID, int* PIDArray, int* _PIDSensorChannels, MotorSensorDriverABC* driver)
{
  // Immediately setup and lock
  mutex = xSemaphoreCreateMutex();
  lock();
  sensor_driver = driver;
  numPID = _numPID;
  motControlType = controlType;
  motProportionType = proportionType;

  for (int i=0; i < numPID; i++)
  {
    motPWMPin[i] = PIDArray[i];
    motPWMChannel[i] = PWMChannelOffset+i;
    motSensorChannel[i] = _PIDSensorChannels[i];

    // Initialize PWM channels
    ledcSetup(motPWMChannel[i], 333, 10);
    ledcAttachPin(motPWMPin[i], motPWMChannel[i]);

    // Initialize PID
    motPID[i] = new PID(motSensorVal+i, motPWM+i, motSetpoint+i, 0, 0, 0, ~((motProportionType & (1<<i))>>i), DIRECT);
    motPID[i]->SetOutputLimits(340,682);
    motPID[i]->SetMode(AUTOMATIC);
    // zero the setpoint, to be safe
    motSetpoint[i] = 0;
  }
  unlock();
}

void inline PIDController::lock()
{
  xSemaphoreTake( mutex, portMAX_DELAY );
}

void inline PIDController::unlock()
{
  xSemaphoreGive( mutex );
}

int PIDController::_update_sensor_values(){
  sensor_driver->update();
  // get positions
  int sensorCount = 0;
  double positions[4] = {0};
  sensor_driver->get_angular_positions(&sensorCount, positions);

  // get velocity
  double velocities[4] = {0};
  sensor_driver->get_angular_velocities(&sensorCount, velocities);

  for (int i = 0; i<numPID; i++)
  {
    // check i-th channel type (0-velocity, 1-position)
    if (motControlType & 1>>i) {
      // position controlled; get the sensor value
      motSensorVal[i] = positions[motSensorChannel[i]];
    } else {
      // velocity controlled; get the sensor value
      motSensorVal[i] = velocities[motSensorChannel[i]];
    }
  }
  return 0;
}

int PIDController::update(){
  lock();
  _update_sensor_values();

  bool running = 1;

  for(int i=0; i<numPID;i++)
  {
    running = running * motPID[i]->Compute();
  }

  _write_PWM_values();
  unlock();
  return running;
}

int PIDController::set_PID_gains(int i, double kp, double ki, double kd){
  if (i >= numPID) return 1;
  lock();
  motPID[i]->SetTunings(kp, ki, kd);
  unlock();
  return 0;
}

int PIDController::get_PID_gains(int i, double *kp, double *ki, double *kd){
  lock();
  *kp = motPID[i]->GetKp();
  *ki = motPID[i]->GetKi();
  *kd = motPID[i]->GetKd();
  unlock();
  return 0;
}

int PIDController::set_PID_setpoint(int i, double setp){
  lock();
  motSetpoint[i] = setp;
  unlock();
  return 0;
}

int PIDController::get_PID_setpoint(int i, double *setp){
  lock();
  *setp = motSetpoint[i];
  unlock();
  return 0;
}

int PIDController::set_sensor_gain(int i, double gain){
  lock();
  sensor_driver->set_angular_gain(motSensorChannel[i],gain);
  unlock();
  return 0;
}

int PIDController::get_sensor_gain(int i, double *gain){
  lock();
  sensor_driver->get_angular_gain(motSensorChannel[i],gain);
  unlock();
  return 0;
}

int PIDController::set_sensor_bias(int i, double bias){
  lock();
  sensor_driver->set_angular_bias(motSensorChannel[i],bias);
  unlock();
  return 0;
}

int PIDController::get_sensor_bias(int i, double *bias){
  lock();
  sensor_driver->get_angular_bias(motSensorChannel[i],bias);
  unlock();
  return 0;
}

int PIDController::set_PID_control_type(uint8_t ctype){
  lock();
  motControlType = ctype;
  // TODO: need to handle when changing PID on-line
  unlock();
  return 0;
}

int PIDController::get_PID_control_type(uint8_t *ctype){
  lock();
  *ctype = motControlType;
  unlock();
  return 0;
}

int PIDController::set_PID_proportion_type(uint8_t ptype){
  lock();
  motProportionType = ptype;
  // TODO: Need to update PID
  // TODO: Need to handle when changing PID on-line
  unlock();
  return 0;
}

int PIDController::get_PID_proportion_type(uint8_t *ptype){
  lock();
  *ptype = motProportionType;
  unlock();
  return 0;
}

int PIDController::zero_PID_sensor(int i, double value){
  lock();
  sensor_driver->reset_position(i, value);
  unlock();
  return 0;
}

int PIDController::get_PID_output(int i, double* output){
  if (i >= numPID) return 1;
  lock();
  *output = motPWM[i];
  unlock();
  return 0;
}

int PIDController::_write_PWM_values(){
  for(int i=0; i<numPID; i++){
    ledcWrite(motPWMChannel[i], motPWM[i]);
  }
  return 0;
}
