#include "UDP_interfaces.h"

UDPSetpoint::UDPSetpoint(int port, int _PIDControllerCount, PIDController** _PIDControllers)
: UDPInterfaceABC(port, _PIDControllerCount, _PIDControllers)
{}

int UDPSetpoint::handle(){
  //if (server.available() == 0) return 0;
  int packet_size = server.parsePacket();
  int read_status;
  if (packet_size){
    read_status = server.read(buffer, 128);
    // return error code if read fails
    //if (read_status) return read_status;

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
      bool request = false; // default to command
      // split out integer index and float setpoint
      for(int c=0; c<packet_size; c++)
      {
        // Index populated
        if (buffer[c] == ',' || buffer[c] == '?'){
          indexEndFound = true;
          intBuf[intIndex] = '\0';
          motorChannelIndex = atoi(intBuf);
          intIndex = 0;
          if (buffer[c] == ',') continue;
          request = true;
          break;
        } else if (indexEndFound){
          if (buffer[c] == '\n') {
            floatBuf[floatIndex] = '\0';
            setpoint = atof(floatBuf);
            floatIndex = 0;
            if (motorChannelIndex < 4*PIDControllerCount) PIDControllers[motorChannelIndex/4]->set_PID_setpoint(motorChannelIndex%4, setpoint);
            indexEndFound = false;
            continue;
          } else{
            floatBuf[floatIndex] = buffer[c];
            floatIndex++;
            continue;
          }
        } else {
          intBuf[intIndex] = buffer[c];
          intIndex++;
          continue;
        }
      }
      // current setpoint requested
      if (request){
        PIDControllers[motorChannelIndex/4]->get_PID_setpoint(motorChannelIndex%4, &setpoint);
        // create a packet containing current setpoint
        server.beginPacket(server.remoteIP(), server.remotePort());
        server.printf("%i,%f", motorChannelIndex, setpoint);
        server.endPacket();
      // new setpoint commanded
      }
      return 0;
    }

    // Have the index and setpoint, now set the PIDController channel
    return PIDControllers[motorChannelIndex/4]->set_PID_setpoint(motorChannelIndex%4, setpoint);
  }
}

UDPSensorBias::UDPSensorBias(int port, int _PIDControllerCount, PIDController** _PIDControllers)
: UDPInterfaceABC(port, _PIDControllerCount, _PIDControllers)
{}

int UDPSensorBias::handle(){
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
    double bias;
    {
      int intIndex = 0;
      char intBuf[8] = {'\0'};
      int floatIndex = 0;
      char floatBuf[16] = {'\0'};
      bool indexEndFound = false;
      bool request = false; // default to command
      // split out integer index and float bias
      for(int c=0; c<packet_size; c++)
      {
        if (buffer[c] == ',' || buffer[c] == '?'){
          indexEndFound = true;
          if (buffer[c] == ',') continue;
          request = true;
          break;
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
      // current bias requested
      if (request){
        PIDControllers[motorChannelIndex/4]->get_sensor_bias(motorChannelIndex%4, &bias);
        // create a packet containing current bias
        server.beginPacket(server.remoteIP(), server.remotePort());
        server.printf("%i,%f", motorChannelIndex, bias);
        server.endPacket();
      // new bias commanded
      }else {
        bias = atof(floatBuf);
      }
    }

    // Have the index and bias, now set the PIDController channel
    return PIDControllers[motorChannelIndex/4]->set_sensor_bias(motorChannelIndex%4, bias);
  }
}


UDPSensorGain::UDPSensorGain(int port, int _PIDControllerCount, PIDController** _PIDControllers)
: UDPInterfaceABC(port, _PIDControllerCount, _PIDControllers)
{}

int UDPSensorGain::handle(){
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
    double gain;
    {
      int intIndex = 0;
      char intBuf[8] = {'\0'};
      int floatIndex = 0;
      char floatBuf[16] = {'\0'};
      bool indexEndFound = false;
      bool request = false; // default to command
      // split out integer index and float gain
      for(int c=0; c<packet_size; c++)
      {
        if (buffer[c] == ',' || buffer[c] == '?'){
          indexEndFound = true;
          if (buffer[c] == ',') continue;
          request = true;
          break;
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
      // current gain requested
      if (request){
        PIDControllers[motorChannelIndex/4]->get_sensor_gain(motorChannelIndex%4, &gain);
        // create a packet containing current gain
        server.beginPacket(server.remoteIP(), server.remotePort());
        server.printf("%i,%f", motorChannelIndex, gain);
        server.endPacket();
      // new gain commanded
      }else {
        gain = atof(floatBuf);
      }
    }

    // Have the index and gain, now set the PIDController channel
    return PIDControllers[motorChannelIndex/4]->set_sensor_gain(motorChannelIndex%4, gain);
  }
}

UDPMotorStateStreaming::UDPMotorStateStreaming(int port, int _PIDControllerCount, PIDController** _PIDControllers)
: UDPInterfaceABC(port, _PIDControllerCount, _PIDControllers)
{}

int UDPMotorStateStreaming::handle(){
  int packet_size = server.parsePacket();
  int read_status;
  if (packet_size){
    read_status = server.read(buffer, 128);
    // return error code if read fails
    if (read_status) return read_status;

    buffer[packet_size] = '\0';

    // Check for proper request
    if (buffer[0] == 'M'){
      server.beginPacket(server.remoteIP(), server.remotePort());
      for(int pidIdx = 0; pidIdx < PIDControllerCount; pidIdx++){
        // get number of PID's in PIDController
        int numPID;
        PIDControllers[pidIdx]->get_num_PID(&numPID);
        // iterate through PID channels
        for(int idx = 0; idx < numPID; idx++){
          double pos, vel, eff;
          // get current PWM ("effort")
          PIDControllers[pidIdx]->get_PID_output(idx, &eff);
          // get current velocity
          PIDControllers[pidIdx]->get_sensor_vel(idx, &vel);
          // get current position
          PIDControllers[pidIdx]->get_sensor_pos(idx, &pos);
          server.printf(
            "%i,%f,%f,%f\n",
            4*pidIdx+idx,
            eff,
            vel,
            pos
          );
        } // Finished a PIDController
      } // Finished all PIDControllers
      server.endPacket();
    } else {
      // echo as rejection
      server.beginPacket(server.remoteIP(), server.remotePort());
      server.printf(buffer);
      server.endPacket();
    }
    server.flush();
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
      bool request = false;

      for(int c = 0; c<packet_size; c++){
        if( (buffer[c] == ',' || buffer[c] == '?')  ){
          if (indexEndFound) {
            gainCount++;
            floatIndex = 0;
            if (buffer[c] == ',') continue;
            request = true;
            break;
          }
          indexEndFound = true;
          continue;
        // index
        } else if (~indexEndFound){
          intBuf[intIndex] = buffer[c];
        // Kp
        } else if(indexEndFound && gainCount == 0){
          floatBufKp[floatIndex] = buffer[c];
          floatIndex++;
          continue;
        // Ki
        } else if(indexEndFound && gainCount == 1){
          floatBufKi[floatIndex] = buffer[c];
          floatIndex++;
          continue;
        // Kd
        } else if(indexEndFound && gainCount == 2){
          floatBufKd[floatIndex] = buffer[c];
          floatIndex++;
          continue;
        }
    }
    motorChannelIndex = atoi(intBuf);
    if (request){
      PIDControllers[motorChannelIndex/4]->get_PID_gains(motorChannelIndex%4, &kp, &ki, &kd);
      server.beginPacket(server.remoteIP(), server.remotePort());
      server.printf("%i,%f,%f,%f", motorChannelIndex, kp, ki, kd);
      server.endPacket();
    } else {
      kp = atof(floatBufKp);
      ki = atof(floatBufKi);
      kd = atof(floatBufKd);
    }
  }
  return PIDControllers[motorChannelIndex/4]->set_PID_gains(motorChannelIndex%4, kp, ki, kd);
 }
}
