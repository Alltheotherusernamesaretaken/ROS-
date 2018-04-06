#ifndef PID_CONTROLLER__H_
#define PID_CONTROLLER__H_
#include<PID_v1.h>
#include<Arduino.h>

#include <motor_sensor_driver_ABC.h>

/// \class PIDController
/// \brief Class for the PID controllers.
///
/// This class implements the interface for the PID controllers.
/// These objects wrap and manage similar sets of PID controllers, hiding their
/// numerous variables behind an object.
class PIDController
{
public:
  /// \brief Constructer method.
  ///
  /// \param controlType Bitmask for motor control type (velocity/position).
  /// \param proportionType Bitmask for motor proportion type (PoE/PoM).
  /// \param PWMChannelOffset Sets the offset of the starting PWM channel.
  /// \param numPID Number of PID channels to create.
  /// \param PIDPins Array of PWM pins for each channel.
  /// \param PIDSensorChannels Array of sensor channels for each PID channel.
  /// \param driver Pointer to MotorSensorDriverABC object to handle sensors
  ///
  PIDController(uint8_t, uint8_t, int, int, int*, int*, MotorSensorDriverABC*);

  /// \brief Updates velocity, PID, and writes PWM.
  ///
  /// This function queries the sensor source. With the new data,
  /// the PID inputs are calculated, using the motControlType and the sensor_driver.
  /// After updating PID inputs, the PID values are computed and written to the
  /// PWM pins.
  ///
  /// If it fails, it returns an integer error code. Successful updates return
  /// 0.
  int update();

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

  /// \brief Sets the sensor gain for the given PID channel's sensor.
  ///
  /// \param channel The index of the PID channel to which to write.
  /// \param gain New desired gain.
  int set_sensor_gain(int,double);
  /// \brief Gets the sensor gain for the give PID channel's sensor.
  ///
  /// \param channel The index of the PID channel from which to read.
  /// \param gain Pointer to which to write current gain.
  int get_sensor_gain(int,double*);

  /// \brief Sets the sensor bias for the given PID channel's sensor.
  ///
  /// \param channel The index of the PID channel to which to write.
  /// \param gain New desired gain.
  int set_sensor_bias(int,double);
  /// \brief Gets the sensor bias for the give PID channel's sensor.
  ///
  /// \param channel The index of the PID channel from which to read.
  /// \param gain Pointer to which to write current gain.
  int get_sensor_bias(int,double*);

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

  /// \brief Zeros the sensor for the PID channel.
  ///
  /// \param channel PID channel index to zero.
  /// \param target Target value to set current position too
  int zero_PID_sensor(int, double target=0.0);

  /// \brief Gets the current PWM command.
  ///
  /// \param channel PID channel index.
  /// \param pwmVal Pointer to put PWM value in.
  int get_PID_output(int, double*);

  /// \brief Sets PID output limits
  ///
  /// \param channel PID channel index.
  /// \param lower PID lower output bound.
  /// \param upper PID upper output bound.
  int set_PID_limits(int, double, double);

private:

  SemaphoreHandle_t mutex; ///< Lock for thread-safe accesses
  int numPID; ///< Number of active PID channels
  int motPWMPin[4]; ///< Array of the pin numbers for each PID channel.
  int motPWMChannel[4]; ///< Array of PWM channels for each PID channel.
  int motSensorChannel[4]; ///< Array of sensor channels for each PID channel.

  PID* motPID[4]; ///< Array of PID objects for each PID channel.

  double motSetpoint[4]; ///< Array of PID setpoints for each PID channel.
  double motSensorVal[4]; ///< Array of PID sensor values for each PID channel.
  double motPWM[4]; ///< Array of PID PWM outputs for each PID channel.

  uint8_t motControlType; ///< Bitmask encoding control types for each PID channel.
  uint8_t motProportionType; ///< Bitmask encoding proportion types for each PID channel.

  MotorSensorDriverABC* sensor_driver;

  /// \brief Queries the sensor object and gets the new sensor values.
  ///
  /// This will be where the inheriting PID controllers most differ.
  int _update_sensor_values();

  /// \brief Writes the PWM values in motPWM to the PWM channels in motPWMChannel.
  int _write_PWM_values();

  void lock();
  void unlock();
};

#endif
