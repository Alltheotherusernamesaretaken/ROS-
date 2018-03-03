#include "udp_functions.h"

// UDP Echo Client variables
const int udpEchoPort = 3333;
WiFiUDP udp_echo;

// UDP Setpoint Client variables
const int udpSetPointPort = 3334;
WiFiUDP udp_setpoint;

// UDP Kp Client variables
const int udpKpPort = 3335;
WiFiUDP udp_kp;

// UDP Ki Client variables
const int udpKiPort = 3336;
WiFiUDP udp_ki;

// UDP Kd Client variables
const int udpKdPort = 3337;
WiFiUDP udp_kd;

void setup_UDP(){
  // UDP echo setup
  udp_echo.begin(udpEchoPort);

  // UDP setpoint
  udp_setpoint.begin(udpSetPointPort);

  // UDP Kp,Ki,Kd
  udp_kp.begin(udpKpPort);
  udp_ki.begin(udpKiPort);
  udp_kd.begin(udpKdPort);
  udp_echo.flush();
  udp_setpoint.flush();
  udp_kp.flush();
  udp_ki.flush();
  udp_kd.flush();
}

void do_UDP(){
  do_UDP_echo();
  do_UDP_setpoint();
  do_UDP_kp();
  do_UDP_ki();
  do_UDP_kd();
}

void do_UDP_echo(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_echo.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_echo.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_echo.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_echo.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_echo.beginPacket(udp_echo.remoteIP(), udp_echo.remotePort());
    udp_echo.printf(buf);
    udp_echo.endPacket();
  }
  udp_echo.flush();
}

void do_UDP_setpoint(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_setpoint.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_setpoint.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_setpoint.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_setpoint.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_setpoint.beginPacket(udp_setpoint.remoteIP(), udp_setpoint.remotePort());
    udp_setpoint.printf(buf);
    udp_setpoint.endPacket();
    cmd_vel = atof(buf);
  }
  udp_setpoint.flush();
}

void do_UDP_kp(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_kp.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_kp.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_kp.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_kp.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_kp.beginPacket(udp_kp.remoteIP(), udp_kp.remotePort());
    udp_kp.printf(buf);
    udp_kp.endPacket();
    Kp = atof(buf);
    PID_ex.SetTunings(Kp,Ki,Kd);
  }
  udp_kp.flush();
}

void do_UDP_ki(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_ki.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_ki.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_ki.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_ki.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_ki.beginPacket(udp_ki.remoteIP(), udp_ki.remotePort());
    udp_ki.printf(buf);
    udp_ki.endPacket();
    Ki = atof(buf);
    PID_ex.SetTunings(Kp,Ki,Kd);
  }
  udp_ki.flush();
}

void do_UDP_kd(){
  static char buf[1024] = {'\0'};
  static int read_status = 0;
  int packet_size = udp_kd.parsePacket();
  // while stuff is in the receive buffer
  if (packet_size >= 1024){
    read_status = udp_kd.read(buf, 1024);
    Serial.println("Too big!");
    Serial.print(buf);
    read_status = udp_kd.read(buf, 1024);
    Serial.println(buf);
  } else if (packet_size) {
    read_status = udp_kd.read(buf, 1024);
    buf[packet_size] = '\0';
    Serial.print("Got: \""); Serial.print(buf); Serial.println("\"");
    // Echo back as ack
    udp_kd.beginPacket(udp_kd.remoteIP(), udp_kd.remotePort());
    udp_kd.printf(buf);
    udp_kd.endPacket();
    Kd = atof(buf);
    PID_ex.SetTunings(Kp,Ki,Kd);
  }
  udp_ki.flush();
}
