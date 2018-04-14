#include "UDP_interfaces.h"

UDPSetpoint::UDPSetpoint(int port, int _PIDControllerCount, PIDController** _PIDControllers)
: UDPInterfaceABC(port, _PIDControllerCount, _PIDControllers)
{}

int UDPSetpoint::handle(){
  int packet_size = server.parsePacket();
  int read_status;
  if (packet_size){
    read_status = server.read(buffer, 128);
    // return error code if read fails
    if (read_status) return read_status;

    buffer[packet_size] = '\0';

    // echo as ACK
    server.beginPacket(server.remoteIP(), server.remotePort());
    server.printf(buffer);
    server.endPacket();

    server.flush();

    // Parse the message now
    int motorChannelIndex;
    double setpoint;
    {
      int intIndex = 0;
      char intBuf[8] = {'\0'};
      int floatIndex = 0;
      char floatBuf[16] = {'\0'};
      bool indexEndFound = false;
      // split out integer index and float setpoint
      for(int c=0; c<packet_size; c++)
      {
        if (buffer[c] == ','){
          indexEndFound = true;
          continue;
        } else if (indexEndFound){
          floatBuf[floatIndex] = buffer[c];
          floatIndex++;
          continue;
        } else {
          intBuf[intIndex] = buffer[c];
          intIndex++;
          continue;
        }
      }
      motorChannelIndex = atoi(intBuf);
      setpoint = atof(floatBuf);
    }

    // Have the index and setpoint, now set the PIDController channel
    return PIDControllers[motorChannelIndex/4]->set_PID_setpoint(motorChannelIndex%4, setpoint);
  }
}


UDPTuningGain::UDPTuningGain(int port, int _PIDControllerCount, PIDController** _PIDControllers)
: UDPInterfaceABC(port, _PIDControllerCount, _PIDControllers)
{}

int UDPTuningGain::handle(){

  int packet_size = server.parsePacket();
  int read_status;
  if (packet_size){
    read_status = server.read(buffer, 128);
    // return error code if read fails
    if (read_status) return read_status;

    buffer[packet_size] = '\0';

    // echo as ACK
    server.beginPacket(server.remoteIP(), server.remotePort());
    server.printf(buffer);
    server.endPacket();

    server.flush();

    int motorChannelIndex;
    double kp;
    double ki;
    double kd;
    {
      int intIndex = 0;
      char intBuf[8] = {'\0'};
      int floatIndex = 0;
      char floatBufKp[16] = {'\0'};
      char floatBufKi[16] = {'\0'};
      char floatBufKd[16] = {'\0'};
      int gainCount = 0;
      bool indexEndFound = false;

      for(int c = 0; c<packet_size; c++){
        if(buffer[c] == ','){
          indexEndFound = true;
          continue;

          // This is for locating the kp, ki, kd values assuming they are sequential and exist. A bit iffy.

        } else if(indexEndFound && gainCount == 0){
          floatBufKp[floatIndex] = buffer[c];
          floatIndex++;
          continue;

        } else if(indexEndFound && gainCount == 1){
          floatBufKi[floatIndex] = buffer[c];
          floatIndex++;
          continue;

        } else if(indexEndFound && gainCount == 2){
          floatBufKd[floatIndex] = buffer[c];
          floatIndex++;
          continue;

        } if (buffer[c+1] == ',' && indexEndFound == true){
          indexEndFound = false;
          gainCount++;
          floatIndex = 0;
        }
    }

    motorChannelIndex = atoi(intBuf);
    kp = atof(floatBufKp);
    ki = atof(floatBufKi);
    kd = atof(floatBufKd);

  }

  return PIDControllers[motorChannelIndex/4]->set_PID_gains(motorChannelIndex%4, kp, ki, kd);

 }
}
