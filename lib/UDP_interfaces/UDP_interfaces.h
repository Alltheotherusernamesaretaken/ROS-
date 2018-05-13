#ifndef UDP_INTERFACES_H_
#define UDP_INTERFACES_H_
#include<Arduino.h>
#include<WiFiUdp.h>

#include "UDP_interface_abc.h"

/// \class Updates PID sensor gains.
///
/// This class updates the sensor gain used to transform the measured
/// plant (motor) output into the same form as the setpoint.
class UDPSensorGain : public UDPInterfaceABC
{
public:
  UDPSensorGain(int, int, PIDController**);

  int handle();
};

/// \class Stream motor state info to requesting client
///
/// This class streams motor sensor data to any client that requests a stream.
class UDPMotorStateStreaming : public UDPInterfaceABC
{
public:
  UDPMotorStateStreaming(int, int, PIDController**);

  int handle();
};

/// \class Updates PID sensor biases.
///
/// This class updates the sensor bias used to transform the measured
/// plant (motor) output into the same form as the setpoint.
class UDPSensorBias : public UDPInterfaceABC
{
public:
  UDPSensorBias(int, int, PIDController**);

  int handle();
};

/// \class Updates PID setpoint.
///
/// Updates the setpoint of PID channels to new values.
class UDPSetpoint : public UDPInterfaceABC
{
public:
  UDPSetpoint(int, int, PIDController**);

  int handle();
};

/// \class Changes proportion type.
///
/// This class updates the proportion type (0/PoE & 1/PoM).
class UDPProportionType : public UDPInterfaceABC
{
public:
  UDPProportionType(int, int, PIDController**);

  int handle();
};

/// \class Changes PID tuning gains.
///
/// This class updates Kp, Ki, and Kd for the PID channels.
class UDPTuningGain : public UDPInterfaceABC
{
public:
  UDPTuningGain(int, int, PIDController**);

  int handle();
};

/// \class Rezero sensor.
///
/// This class resets sensor offset for the desired PID channel. It clears the
/// encoder counter for quadrature sensors and updates the offset for analog.
class UDPZeroReset : public UDPInterfaceABC
{
public:
  UDPZeroReset(int, int, PIDController**);

  int handle();
};

/// \class Stops motors and reboots the ESP32.
///
/// This class will shutdown all PID channels and initiate a reboot.
class UDPReboot : public UDPInterfaceABC
{
public:
  UDPReboot(int, int, PIDController**);

  int handle();
};

/// \class Sets PID type.
///
/// Changes PID channel control type (0/velocity 1/position).
class UDPControlType : public UDPInterfaceABC
{
public:
  UDPControlType(int, int, PIDController**);

  int handle();
};

#endif
