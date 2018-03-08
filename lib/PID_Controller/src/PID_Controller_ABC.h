#ifndef PID_CONTROLLER_ABC_H_
#define PID_CONTROLLER_ABC_H_
#include<PID_v1.h>
#include<Arduino.h>

/// \class PIDControllerABC
/// \brief Abstract Base Class for the PID controllers.
///
/// This abstract base class implements the interface for the PID controllers.
/// These objects wrap and manage similar sets of PID controllers, hiding their
/// numerous variables behind an object. Each class inheriting from this ABC
/// implements a different sensor driver.
class PIDControllerABC
{
public:
  /// \brief Constructer method.
  ///
  /// \param controlType Bitmask for motor control type (velocity/position).
  /// \param proportionType Bitmask for motor proportion type (PoE/PoM).
  /// \param pwmChannelOffset Sets the offset of the starting PWM channel.
  /// \param numPID Number of PID channels to create.
  /// \param PIDPins Array of PWM pins for each channel.
  ///
  PIDControllerABC(uint8_t, uint8_t, int, int, int*);

  /// \brief Updates velocity, PID, and writes PWM.
  ///
  /// This function queries the sensor source. With the new data,
  /// the PID inputs are calculated, using the motSensorGain and motControlType.
  /// After updating PID inputs, the PID values are computed and written to the
  /// PWM pins.
  ///
  /// If it fails, it returns an integer error code. Successful updates return
  /// 0.
  virtual int update();

  /// \brief Sets the PID gains for the given PID channel.
  ///
  /// \param channel The index of the PID channel to update.
  /// \param Kp Proportional gain.
  /// \param Ki Integral gain.
  /// \param Kd Derivative gain.
  int set_PID_gains(int, double, double, double);
  /// \brief Gets the PID gains for the given PID channel.
  ///
  /// \param channel The index of the PID channel from which to read values.
  /// \param Kp Pointer where Proportional gain is written to.
  /// \param Ki Pointer where Integral gain is written to.
  /// \param Kd Pointer where Derivative gain is written to.
  int get_PID_gains(int,double*, double*, double*);

  /// \brief Sets the PID setpoint for the given PID channel.
  ///
  /// \param channel The index of the PID channel to which to write.
  /// \param setpoint New desired setpoint.
  int set_PID_setpoint(int,double);
  /// \brief Gets the PID setpoint for the given PID channel.
  ///
  /// \param channel The index of the PID channel from which to read.
  /// \param setpoint Pointer to which to write current setpoint.
  int get_PID_setpoint(int,double*);

  /// \brief Sets the sensor gain for the given PID channel.
  ///
  /// \param channel The index of the PID channel to which to write.
  /// \param gain New desired gain.
  int set_sensor_gain(int,double);
  /// \brief Gets the PID setpoint for the give PID channel.
  ///
  /// \param channel The index of the PID channel from which to read.
  /// \param gain Pointer to which to write current gain.
  int get_sensor_gain(int,double*);

  /// \brief Sets the bitmask for the PID control types.
  ///
  /// \param bitmask Bitmask encoding control types (0/velocity and 1/position).
  int set_PID_control_type(uint8_t);
  /// \brief Gets the bitmask for the PID control types.
  ///
  /// \param bitmask Pointer to bitmask encoding control types (0/velocity and 1/position).
  int get_PID_control_type(uint8_t*);

    /// \brief Sets the bitmask for the PID proportion types.
    ///
    /// \param bitmask Bitmask encoding proportion types (0/PoE and 1/PoM).
  int set_PID_proportion_type(uint8_t);
  /// \brief Gets the bitmask for the PID proportion types.
  ///
  /// \param bitmask Pointer to bitmask encoding proportion types (0/PoE and 1/PoM).
  int get_PID_proportion_type(uint8_t*);

private:
  int motPWMPin[8]; ///< Array of the pin numbers for each PID channel.
  int motPWMChannel[8]; ///< Array of PWM channels for each PID channel.
  int motSensorChannel[8]; ///< Array of sensor channels for each PID channel.

  PID motPID[8]; ///< Array of PID objects for each PID channel.

  double motSetpoint[8]; ///< Array of PID setpoints for each PID channel.
  double motSensorVal[8]; ///< Array of PID sensor values for each PID channel.
  double motPWM[8]; ///< Array of PID PWM outputs for each PID channel.
  double motSensorGain[8]; ///< Array of PID sensor gains for each PID channel.

  uint8_t motControlType; ///< Bitmask encoding control types for each PID channel.
  uint8_t motProportionType; ///< Bitmask encoding proportion types for each PID channel.

  /// \brief Queries the sensor object and gets the new sensor values.
  virtual int _update_sensor_values();

  /// \brief Writes the PWM values in motPWM to the PWM channels in motPWMChannel.
  int _write_PWM_values();
};

#endif
