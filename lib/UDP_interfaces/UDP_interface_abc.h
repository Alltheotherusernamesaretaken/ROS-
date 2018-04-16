#ifndef UDP_INTERFACE_ABC_H_
#define UDP_INTERFACE_ABC_H_
#include<Arduino.h>
#include<WiFiUdp.h>

#include "PID_Controller.h"

/// \class This abstract base class implements the UDP interface.
///
/// This class implements the rudimentary components needed across the many
/// UDP servers. It spins up a UDP server on the given port. It also maintains
/// an array containing pointers to PIDController objects it must interface
/// with to function. Each PIDController manages up to 4 channels. Channels are
/// referenced by the UDP interface via a single index formed from
/// concatenating the PIDController indexes.
///
/// Examples:
/// Index 1 corresponds to the 1st PIDController's 1st channel.
/// Index 7 corresponds to the 2nd PIDController's 3rd channel.
class UDPInterfaceABC
{
public:
  /// \brief Constructor method.
  ///
  /// \param port The port number to listen too.
  /// \param PIDcontrollerCount The number of PID controllers present.
  /// \param PIDcontrollers An array of the PID controllers.
  UDPInterfaceABC(int port,int _PIDControllerCount,PIDController** _PIDControllers){
    PIDControllerCount = (_PIDControllerCount >= 4) ? 4 : _PIDControllerCount;
    for (int i=0;i<PIDControllerCount;i++)
    {
      PIDControllers[i] = _PIDControllers[i];
    }
    server.begin(port);
  }

  /// \brief Handles incoming/outcoming messages
  virtual int handle();

protected:
  int port; ///< Stores the port number.
  WiFiUDP server; ///< The actual UDP server.
  char buffer[128] = {'\0'}; ///< A character buffer for reading packets.
  int PIDControllerCount;
  PIDController* PIDControllers[4];
};

#endif
