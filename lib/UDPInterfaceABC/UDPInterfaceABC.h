#include <vector>

#include <Arduino.h>
#include <WiFiUdp.h>

#include "MotorInterfaceABC.h"


class UDPInterface
{
protected:
    /// \brief server for UDP comms
    WiFiUDP server;
    /// \brief port number for running the server
    int port;
    /// \brief buffer for message parsing
    char buffer[256];
    /// \brief vector containing motor objects
    std::vector<MotorInterfaceABC> motors;

    /// \brief This method handles reading in a new message into the buffer
    virtual int read();
    /// \brief This method parses the message and generates the outgoing message
    virtual int parse();
    /// \brief This method responds to the client
    virtual int respond();

public:
    UDPInterface(int _port)
    {
        port = _port;
        // start the server listening on the port
        server.begin(port);
        // flush it just in case
        server.flush();
    }
    /// \brief This function handles reading, parsing, and responding
    virtual int handle();

};