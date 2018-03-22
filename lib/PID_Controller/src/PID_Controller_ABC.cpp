#include "PID_Controller_ABC.h"

PIDControllerABC::PIDControllerABC(uint8_t controlType, uint8_t proportionType, int PWMChannelOffset, int numPID, int* PIDArray)
{
  // Immediately setup and lock
  mutex = xSemaphoreCreateMutex();
  lock();

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
