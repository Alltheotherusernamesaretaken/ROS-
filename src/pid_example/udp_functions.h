#ifndef UDP_FUNCTIONS_H
#define UDP_FUNCTIONS_H

#include<Arduino.h>
#include<WiFiUdp.h>
#include<PID_v1.h>

// These are defined elsewhere
extern double cmd_vel, velocity, pwm;
extern double Ki, Kp, Kd;
extern PID PID_ex;

// This is called to spin up everything
void setup_UDP();

void do_UDP();

// UDP Echo Client variables
extern const int udpEchoPort;
extern WiFiUDP udp_echo;
// UDP Declarations
void do_UDP_echo();

// UDP Setpoint Client variables
extern const int udpSetPointPort;
extern WiFiUDP udp_setpoint;
void do_UDP_setpoint();

// UDP Kp Client variables
extern const int udpKpPort;
extern WiFiUDP udp_kp;
void do_UDP_kp();

// UDP Ki Client variables
extern const int udpKiPort;
extern WiFiUDP udp_ki;
void do_UDP_ki();

// UDP Kd Client variables
extern const int udpKdPort;
extern WiFiUDP udp_kd;
void do_UDP_kd();

#endif
