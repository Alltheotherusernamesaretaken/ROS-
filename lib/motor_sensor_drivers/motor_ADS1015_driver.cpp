#include "motor_ADS1015_driver.h"
#include <Wire.h>

MotorADS1015Driver::MotorADS1015Driver(int address)
: MotorSensorDriverABC(), ads(address)
{
  numSensors = 4;
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

LinearActuatorSensorDriver::LinearActuatorSensorDriver(double* _armLengths)
: MotorADS1015Driver()
{
  for(int i=0; i<5; i++)
  {
    armLengths[i] = _armLengths[i];
  }
}

LinearActuatorSensorDriver::LinearActuatorSensorDriver(int address, double* _armLengths)
: MotorADS1015Driver(address)
{
  for(int i=0; i<5; i++)
  {
    armLengths[i] = _armLengths[i];
  }
}

int LinearActuatorSensorDriver::update(){
    int new_adcs[4];
    unsigned int new_timestamps[4];
    // read adc's
    for(int i = 0; i<numSensors;i++)
    {
      new_timestamps[i] = millis();
      new_adcs[i] = ads.readADC_SingleEnded(i);
    }
    // convert adc values to angles
    double cur_angles[4];
    cur_angles[0] = angle_gain[0]*new_adcs[0]+angle_bias[0];
    cur_angles[1] = angle_gain[1]*new_adcs[1]+angle_bias[1];
    cur_angles[2] = angle_gain[2]*new_adcs[2]+angle_bias[2];
    cur_angles[3] = angle_gain[3]*new_adcs[3]+angle_bias[3];
    // grab previous positions
    double prev_positions[4];
    prev_positions[0] = angle_positions[0];
    prev_positions[1] = angle_positions[1];
    prev_positions[2] = angle_positions[2];
    prev_positions[3] = angle_positions[3];
    // Generate intermediate values for arm calculations
    double VAL_LEFT = asin(armLengths[1]/armLengths[0]*sin(cur_angles[0]));
    double VAL_RIGHT = asin(armLengths[1]/armLengths[0]*sin(cur_angles[1]));
    // back actuators, BC
    // left
    angle_positions[0] = armLengths[0]*sin(cur_angles[0]-VAL_LEFT)/sin(cur_angles[0]);
    // right
    angle_positions[1] = armLengths[0]*sin(cur_angles[1]-VAL_RIGHT)/sin(cur_angles[1]);
    // front actuators, EO
    // Left
    angle_positions[2] = (
      (
        armLengths[2]*sin(
          cur_angles[2]-asin(
            0.5/armLengths[2]*(
              armLengths[0]*sin(
                cur_angles[0]-VAL_LEFT-cur_angles[2]
              )-2*sin(
                cur_angles[2]
              )*(
                armLengths[3]+armLengths[0]*cos(
                  cur_angles[0]-VAL_LEFT
                )
              )+armLengths[0]*sin(
                cur_angles[0]-VAL_LEFT+cur_angles[2]
              )
            )
          )
        )+armLengths[0]*sin(
          cur_angles[0]-VAL_LEFT
        )
      )/sin(
        cur_angles[2]
      )
    );
    // Right
    angle_positions[3] = (
      (
        armLengths[2]*sin(
          cur_angles[3]-asin(
            0.5/armLengths[2]*(
              armLengths[0]*sin(
                cur_angles[1]-VAL_RIGHT-cur_angles[3]
              )-2*sin(
                cur_angles[3]
              )*(
                armLengths[3]+armLengths[0]*cos(
                  cur_angles[1]-VAL_RIGHT
                )
              )+armLengths[0]*sin(
                cur_angles[1]-VAL_RIGHT+cur_angles[3]
              )
            )
          )
        )+armLengths[0]*sin(
          cur_angles[1]-VAL_RIGHT
        )
      )/sin(
        cur_angles[3]
      )
    );
    // Store actuator positions and velocities
    for(int i=0; i<numSensors; i++)
    {
      double dt = ((double)new_timestamps[i] - prev_timestamps[i])/1000.0;

      angle_velocities[i] = (angle_positions[i]-prev_positions[i])/dt;

      prev_timestamps[i] = new_timestamps[i];
      prev_adcs[i] = new_adcs[i];
    }

    return 0;
}
