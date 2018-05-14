#ifndef ROBOT_CONFIGS
#define ROBOT_CONFIGS

#include <Arduino.h>
#include "PID_Controller.h"
#include "UDP_interfaces.h"
#include "motor_spi_encoder_driver.h"
#include "motor_ADS1015_driver.h"
#include <Wire.h>

// Logic Level Shifter Channels
// 1:
// 2: SS1 N/C
// 3: SS2 N/C
// 4: SS3 16
// 5: SS4 4
// 6: MOSI 23
// 7: MISO 19
// 8: SCLK 18
const char* hostname = "dumper";

// Configs
//#define SPI_PINS 14,27,13,15
#define MOSI 23
#define MISO 19
#define SCLK 18
#define SS 5

#define NUM_PID 2

#define SCL 22
#define SDA 21
//#define I2C_PINS 21,22
// UDP
#define SETPOINT_PORT 3233

// SPI 1
#define SPI_1_NUM 0
#define SPI_1_DRIVE_PINS 0 //15,0,2,4
// SPI 1 Sensor Gains
  #define SPI_1_1_GAIN 0
  #define SPI_1_1_BIAS 511
  #define SPI_1_2_GAIN 0
  #define SPI_1_2_BIAS 511
  #define SPI_1_3_GAIN 0 //1/1024/4*2*3.14
  #define SPI_1_3_BIAS 511
  #define SPI_1_4_GAIN 0
  #define SPI_1_4_BIAS 511
// SPI 1 PID
#define SPI_1_CONTROL_TYPE 0b00001111
#define SPI_1_PROPORTION_TYPE 0b00001111
#define SPI_1_PWM_OFFSET 0
#define SPI_1_PWM_CHANNELS 14, 27, 26, 25
#define SPI_1_SENSOR_CHANNELS 0,1,2,3
  // SPI 1 PID Gains
  #define SPI_1_1_KP 1
  #define SPI_1_1_KI 0
  #define SPI_1_1_KD 0
  #define SPI_1_2_KP 1
  #define SPI_1_2_KI 0
  #define SPI_1_2_KD 0
  #define SPI_1_3_KP 1 //2.8195
  #define SPI_1_3_KI 0 //44.2844
  #define SPI_1_3_KD 0
  #define SPI_1_4_KP 1
  #define SPI_1_4_KI 0
  #define SPI_1_4_KD 0

// SPI 2
#define SPI_2_NUM 2
#define SPI_2_DRIVE_PINS 4, 16 //15
// SPI 2 Sensor Gains
  #define SPI_2_1_GAIN (1.0f/1024.0f/4.0f*2.0f*3.14f)
  #define SPI_2_1_BIAS 0
  #define SPI_2_2_GAIN (1.0f/1024.0f/4.0f*2.0f*3.14f)
  #define SPI_2_2_BIAS 0
// SPI 2 PID
#define SPI_2_CONTROL_TYPE 0b00001111
#define SPI_2_PROPORTION_TYPE 0b00001111
#define SPI_2_PWM_OFFSET 4
#define SPI_2_PWM_CHANNELS 5, 17
#define SPI_2_SENSOR_CHANNELS 0,1
  // SPI 2 PID Gains
  #define SPI_2_1_KP 25 //20
  #define SPI_2_1_KI 0.1 //12.5
  #define SPI_2_1_KD 6
  #define SPI_2_2_KP 25
  #define SPI_2_2_KI 0.1
  #define SPI_2_2_KD 6

// Gravity Compensation Values
#define GRAVITY_GAIN 24.289f
#define GRAVITY_BIAS -0.65521f


// SPI objects
// SPI 1 - Drive
#ifdef SPI_1_NUM
int numberOfEncoders_1 = SPI_1_NUM;
int encoderSelectPinArray_1[] = { SPI_1_DRIVE_PINS };
MotorSPIEncoderDriver SPI_enc_1(numberOfEncoders_1, encoderSelectPinArray_1);
#endif
// SPI 2- Accessories
#ifdef SPI_2_NUM
int numberOfEncoders_2 = SPI_2_NUM;
int encoderSelectPinArray_2[] = { SPI_2_DRIVE_PINS };
MotorSPIEncoderDriver SPI_enc_2(numberOfEncoders_2, encoderSelectPinArray_2);
//SPIRollingAverageEncoderDriver SPI_enc_2(numberOfEncoders_2, encoderSelectPinArray_2, 2);
#endif


// PID objects
// SPI PID 1 - Drive
uint8_t SPIcontrolType_1 = SPI_1_CONTROL_TYPE; // All Velocity
uint8_t SPIproportionType_1 = SPI_1_PROPORTION_TYPE; // All PoE
int SPIPWMChannelOffset_1 = SPI_1_PWM_OFFSET; // First PID so no offset
int SPInumPID_1 = 4; // 4 PID
int SPIPWMpins_1[ 4 ] = { SPI_1_PWM_CHANNELS }; // The four PWM pins
int SPIPIDSensorChannels_1[ 4 ] = { SPI_1_SENSOR_CHANNELS };
PIDController spi_pid_1(
  SPIcontrolType_1,
  SPIproportionType_1,
  SPIPWMChannelOffset_1,
  SPInumPID_1,
  SPIPWMpins_1,
  SPIPIDSensorChannels_1,
  &SPI_enc_1
);
// SPI PID 2 - Accessories
uint8_t SPIcontrolType_2 = SPI_2_CONTROL_TYPE; // All Velocity
uint8_t SPIproportionType_2 = SPI_2_PROPORTION_TYPE; // All PoE
int SPIPWMChannelOffset_2 = SPI_2_PWM_OFFSET; // First PID so no offset
int SPIPIDSensorChannels_2[ SPI_2_NUM] = { SPI_2_SENSOR_CHANNELS };
int SPIPWMpins_2[ SPI_2_NUM] = { SPI_2_PWM_CHANNELS }; // The four PWM pins
int SPInumPID_2 = SPI_2_NUM; // 4 PID
GravityCompPID spi_pid_2(
  SPIcontrolType_2,
  SPIproportionType_2,
  SPIPWMChannelOffset_2,
  SPInumPID_2,
  SPIPWMpins_2,
  SPIPIDSensorChannels_2,
  &SPI_enc_2,
  GRAVITY_GAIN,
  GRAVITY_BIAS
);

// UDP
PIDController* PIDArray[2] = {&spi_pid_1, &spi_pid_2};
UDPSetpoint setpointServer(SETPOINT_PORT,2,PIDArray);

// init sensors and PID
inline void init() {
  #ifdef SPI_1_NUM
  // SPI PID 1
  {
    SPI_enc_1.begin();
    // 1
    spi_pid_1.set_sensor_gain(0,SPI_1_1_GAIN);
    spi_pid_1.set_sensor_bias(0,SPI_1_1_BIAS);
    spi_pid_1.set_PID_gains(0, SPI_1_1_KP, SPI_1_1_KI, SPI_1_1_KD);
    spi_pid_1.set_PID_setpoint(0, 511);
    // 2
    spi_pid_1.set_sensor_gain(1,SPI_1_2_GAIN);
    spi_pid_1.set_sensor_bias(1,SPI_1_2_BIAS);
    spi_pid_1.set_PID_gains(1, SPI_1_2_KP, SPI_1_2_KI, SPI_1_2_KD);
    spi_pid_1.set_PID_setpoint(1, 511);
    // 3
    spi_pid_1.set_sensor_gain(2,SPI_1_3_GAIN);
    spi_pid_1.set_sensor_bias(2,SPI_1_3_BIAS);
    spi_pid_1.set_PID_gains(2, SPI_1_3_KP, SPI_1_3_KI, SPI_1_3_KD);
    spi_pid_1.set_PID_setpoint(2, 511);
    // 4
    spi_pid_1.set_sensor_gain(3,SPI_1_4_GAIN);
    spi_pid_1.set_sensor_bias(3,SPI_1_4_BIAS);
    spi_pid_1.set_PID_gains(3, SPI_1_4_KP, SPI_1_4_KI, SPI_1_4_KD);
    spi_pid_1.set_PID_setpoint(3, 511);
  }
  #endif
  #ifdef SPI_2_NUM
  // SPI PID 2
  {
    SPI_enc_2.begin();
    // 1
    spi_pid_2.set_sensor_gain(0,SPI_2_1_GAIN);
    spi_pid_2.set_sensor_bias(0,SPI_2_1_BIAS);
    spi_pid_2.set_PID_gains(0, SPI_2_1_KP, SPI_2_1_KI, SPI_2_1_KD);
    spi_pid_2.set_PID_setpoint(0, 0);
    // 2
    spi_pid_2.set_sensor_gain(1,SPI_2_2_GAIN);
    spi_pid_2.set_sensor_bias(1,SPI_2_2_BIAS);
    spi_pid_2.set_PID_gains(1, SPI_2_2_KP, SPI_2_2_KI, SPI_2_2_KD);
    spi_pid_2.set_PID_setpoint(1, 0);
  }
  #endif
  setpointServer.begin();
}
// update PIDs
inline void update(){
  #ifdef SPI_1_NUM
  spi_pid_1.update();
  #endif
  #ifdef SPI_2_NUM
  spi_pid_2.update();
  #endif
}

// handle UDP servers
inline void updateUDP(){
  setpointServer.handle();
}

#endif
