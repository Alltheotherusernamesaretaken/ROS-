#ifndef UDP_INTERFACE_ABC_H_
#define UDP_INTERFACE_ABC_H_
#include<Arduino.h>
#include<WiFiUdp.h>

/// \class This abstract base class implements the UDP interface.
///
/// This class implements the rudimentary components needed across the many
/// UDP servers. Since the ESP32 is dual-core, the WiFi process (UDP included)
/// are run on one core while hardware interaction (PID, I2C, etc.) run in
/// parallel on the other core. Thus, the UDP interfaces need to be thread-safe
/// when reading/writing data. So, a SemaphoreHandle_t is expected at
/// initialization. This is used in child classes to access data in a
/// thread-safe manner.
class UDPInterfaceABC
{
public:
  /// \brief Constructor method.
  ///
  /// \param port The port number to listen too.
  /// \param lock A pointer to the SemaphoreHandle_t to use as a lock.
  UDPInterfaceABC(int, SemaphoreHandle_t*);

  /// \brief Handles incoming/outcoming messages
  virtual int handle();

private:
  int port; ///< Stores the port number.
  WiFiUDP server; ///< The actual UDP server.
  char buffer[128]; ///< A character buffer for reading packets.
  SemaphoreHandle_t* lock; ///< A pointer to the semaphore to use for data transfer.
};

#endif
