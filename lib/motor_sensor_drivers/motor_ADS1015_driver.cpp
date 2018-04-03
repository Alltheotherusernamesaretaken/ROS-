#include "motor_ADS1015_driver.h"
#include <Wire.h>

MotorADS1015Driver::MotorADS1015Driver(int address)
: MotorSensorDriverABC(), ads(address)
{
  numSensors = 4;
  Wire.begin(21,22);
}

MotorADS1015Driver::MotorADS1015Driver()
: MotorADS1015Driver(ADS1015_ADDRESS)
{}

int MotorADS1015Driver::update(){
    int new_adcs[4];
    unsigned int new_timestamps[4];

    for(int i = 0; i<numSensors;i++)
    {
      new_timestamps[i] = millis();
      new_adcs[i] = ads.readADC_SingleEnded(i);
    }

    for(int i=0; i<numSensors; i++)
    {
      angle_positions[i] = ((double) new_adcs[i])*angle_gain[i]+angle_bias[i];
      double dt = ((double)new_timestamps[i] - prev_timestamps[i])/1000.0;

      angle_velocities[i] = angle_gain[i]*((double) (new_adcs[i]-prev_adcs[i]))/dt;

      prev_timestamps[i] = new_timestamps[i];
      prev_adcs[i] = new_adcs[i];
    }
    return 0;
}
