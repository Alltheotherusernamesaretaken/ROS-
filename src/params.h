#ifndef PARAMS_ESP32_ROBOT_DRIVER_H_
#define PARAMS_ESP32_ROBOT_DRIVER_H_
/*
 * Contains parameters for robot firmware
 * 
 * 
 * 
 * 
 * 
 * 
 */


// Set ssid and password here
const char* ssid = "Team_13";
const char* password = "StateSpaceRobotics";
const char* hostname = "Robot32";

// Physical parameters
double base_width = 1.0; // width of wheelbase in meters
double wheel_rad = 1.0; // radius of wheels in meters

// Define type of motor control and necessary parameters
#define QUAD_PWM // QUAD_PWM, DUAL_PWM, QUAD_ROBOCLAW, DUAL_ROBOCLAW, QUAD_ODRIVE, DUAL_ODRIVE

// set appropriate motor parameters and objects
#ifdef QUAD_PWM // params for QUAD_PWM motor control
/*
 * This control mode presumes four independent PWM/DIR motors with quadrature encoders.
 * 
 */
// PWM pins
const int FL_PWM = 0;
const int FR_PWM = 0;
const int BL_PWM = 0;
const int BR_PWM = 0;
// DIR pins
const int L_DIR_1 = 0;
const int L_DIR_2 = 0;
const int R_DIR_1 = 0;
const int R_DIR_2 = 0;
// PWM range
const int PWM_MAX = 255;
// encoder counts per revolution
const int cpr = 1024;
// encoder pins
const int FL_A = 0;
const int FL_B = 0;
const int FR_A = 0;
const int FR_B = 0;
const int BL_A = 0;
const int BL_B = 0;
const int BR_A = 0;
const int BR_B = 0;
// PID
const int KP = 0;
const int KI = 0;
const int KD = 0;

#include<quad.h>

#endif // end of QUAD_PWM

#endif