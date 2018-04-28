#ifndef ROBOT_CONFIGS
#define ROBOT_CONFIGS

#include <Arduino.h>
#include "PID_Controller.h"
#include "UDP_interfaces.h"
#include "motor_spi_encoder_driver.h"
#include "motor_ADS1015_driver.h"
#include <Wire.h>

// Configs
//#define SPI_PINS 14,27,13,15
#define MOSI 23
#define MISO 19
#define SCLK 18
#define SS 5

#define SCL 22
#define SDA 21
//#define I2C_PINS 21,22
// SPI 1
#define SPI_1_NUM 4
#define SPI_1_DRIVE_PINS 15,2,0,4
// SPI 1 Sensor Gains
  #define SPI_1_1_GAIN 1
  #define SPI_1_1_BIAS 1
  #define SPI_1_2_GAIN 1
  #define SPI_1_2_BIAS 1
  #define SPI_1_3_GAIN 1
  #define SPI_1_3_BIAS 1
  #define SPI_1_4_GAIN 1
  #define SPI_1_4_BIAS 1
// SPI 1 PID
#define SPI_1_CONTROL_TYPE 0b00000000
#define SPI_1_PROPORTION_TYPE 0b00000000
#define SPI_1_PWM_OFFSET 0
#define SPI_1_PWM_CHANNELS 13,12,14,27
#define SPI_1_SENSOR_CHANNELS 0,1,2,3
  // SPI 1 PID Gains
  #define SPI_1_1_KP 1
  #define SPI_1_1_KI 1
  #define SPI_1_1_KD 1
  #define SPI_1_2_KP 1
  #define SPI_1_2_KI 1
  #define SPI_1_2_KD 1
  #define SPI_1_3_KP 1
  #define SPI_1_3_KI 1
  #define SPI_1_3_KD 1
  #define SPI_1_4_KP 1
  #define SPI_1_4_KI 1
  #define SPI_1_4_KD 1

// SPI 2
#define SPI_2_NUM 2
#define SPI_2_DRIVE_PINS 16,17
// SPI 2 Sensor Gains
  #define SPI_2_1_GAIN 1
  #define SPI_2_1_BIAS 1
  #define SPI_2_2_GAIN 1
  #define SPI_2_2_BIAS 1
// SPI 2 PID
#define SPI_2_CONTROL_TYPE 0b00000000
#define SPI_2_PROPORTION_TYPE 0b00000000
#define SPI_2_PWM_OFFSET 4
#define SPI_2_PWM_CHANNELS 3,1
#define SPI_2_SENSOR_CHANNELS 0,1
  // SPI 2 PID Gains
  #define SPI_2_1_KP 1
  #define SPI_2_1_KI 1
  #define SPI_2_1_KD 1
  #define SPI_2_2_KP 1
  #define SPI_2_2_KI 0
  #define SPI_2_2_KD 0

// ADC 1
#define ACTUATOR_ARM_LENGTHS
#define ADC_1_1_GAIN 1
#define ADC_1_1_BIAS 1
#define ADC_1_2_GAIN 1
#define ADC_1_2_BIAS 1
#define ADC_1_3_GAIN 1
#define ADC_1_3_BIAS 1
#define ADC_1_4_GAIN 1
#define ADC_1_4_BIAS 1

// ADC 1 PID
#define ADC_1_NUM_PID 4
#define ADC_1_CONTROL_TYPE 0b00001111
#define ADC_1_PROPORTION_TYPE 0b00001111
#define ADC_1_PWM_OFFSET 6
#define ADC_1_PWM_CHANNELS 26,25,33,32
#define ADC_1_SENSOR_CHANNELS 0,1,2,3
  // ADC 1 PID Gains
  #define ADC_1_1_KP 1
  #define ADC_1_1_KI 1
  #define ADC_1_1_KD 1
  #define ADC_1_2_KP 1
  #define ADC_1_2_KI 1
  #define ADC_1_2_KD 1
  #define ADC_1_3_KP 1
  #define ADC_1_3_KI 1
  #define ADC_1_3_KD 1
  #define ADC_1_4_KP 1
  #define ADC_1_4_KI 1
  #define ADC_1_4_KD 1


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
#endif
// ADC objects
// ADC 1 - linear actuators
#ifdef ACTUATOR_ARM_LENGTHS
double actautorArmArray[] = { ACTUATOR_ARM_LENGTHS };
LinearActuatorSensorDriver ADC_pot_1( actautorArmArray );
#endif

// PID objects
// SPI PID 1 - Drive
uint8_t SPIcontrolType_1 = SPI_1_CONTROL_TYPE; // All Velocity
uint8_t SPIproportionType_1 = SPI_1_PROPORTION_TYPE; // All PoE
int SPIPWMChannelOffset_1 = SPI_1_PWM_OFFSET; // First PID so no offset
int SPInumPID_1 = SPI_1_NUM; // 4 PID
int SPIPWMpins_1[ SPI_1_NUM ] = { SPI_1_PWM_CHANNELS }; // The four PWM pins
int SPIPIDSensorChannels_1[ SPI_1_NUM ] = { SPI_1_SENSOR_CHANNELS };
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
int SPInumPID_2 = SPI_2_NUM; // 4 PID
int SPIPWMpins_2[ SPI_2_NUM ] = { SPI_2_PWM_CHANNELS }; // The four PWM pins
int SPIPIDSensorChannels_2[ SPI_2_NUM ] = { SPI_2_SENSOR_CHANNELS };
PIDController spi_pid_2(
  SPIcontrolType_2,
  SPIproportionType_2,
  SPIPWMChannelOffset_2,
  SPInumPID_2,
  SPIPWMpins_2,
  SPIPIDSensorChannels_2,
  &SPI_enc_2
);
// ADC PID 3 - Actuators
uint8_t ADCcontrolType_1 = ADC_1_CONTROL_TYPE; // All Velocity
uint8_t ADCproportionType_1 = ADC_1_PROPORTION_TYPE; // All PoE
int ADCPWMChannelOffset_1 = ADC_1_PWM_OFFSET; // First PID so no offset
int ADCnumPID_1 = ADC_1_NUM_PID; // 4 PID
int ADCPWMpins_1[ ADC_1_NUM_PID ] = { ADC_1_PWM_CHANNELS }; // The four PWM pins
int ADCPIDSensorChannels_1[ ADC_1_NUM_PID ] = { ADC_1_SENSOR_CHANNELS };
PIDController adc_pid_1(
  ADCcontrolType_1,
  ADCproportionType_1,
  ADCPWMChannelOffset_1,
  ADCnumPID_1,
  ADCPWMpins_1,
  ADCPIDSensorChannels_1,
  &ADC_pot_1
);
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
    spi_pid_1.set_PID_setpoint(0, 0);
    // 2
    spi_pid_1.set_sensor_gain(1,SPI_1_2_GAIN);
    spi_pid_1.set_sensor_bias(1,SPI_1_2_BIAS);
    spi_pid_1.set_PID_gains(1, SPI_1_2_KP, SPI_1_2_KI, SPI_1_2_KD);
    spi_pid_1.set_PID_setpoint(1, 0);
    // 3
    spi_pid_1.set_sensor_gain(2,SPI_1_3_GAIN);
    spi_pid_1.set_sensor_bias(2,SPI_1_3_BIAS);
    spi_pid_1.set_PID_gains(2, SPI_1_3_KP, SPI_1_3_KI, SPI_1_3_KD);
    spi_pid_1.set_PID_setpoint(2, 0);
    // 4
    spi_pid_1.set_sensor_gain(3,SPI_1_4_GAIN);
    spi_pid_1.set_sensor_bias(3,SPI_1_4_BIAS);
    spi_pid_1.set_PID_gains(3, SPI_1_4_KP, SPI_1_4_KI, SPI_1_4_KD);
    spi_pid_1.set_PID_setpoint(3, 0);
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
  #ifdef ACTUATOR_ARM_LENGTHS
  // ADC PID 1
  {
    // 1
    adc_pid_1.set_sensor_gain(0,ADC_1_1_GAIN);
    adc_pid_1.set_sensor_bias(0,ADC_1_1_BIAS);
    adc_pid_1.set_PID_gains(0, ADC_1_1_KP, ADC_1_1_KI, ADC_1_1_KD);
    adc_pid_1.set_PID_setpoint(0, 0);
    // 2
    adc_pid_1.set_sensor_gain(1,ADC_1_2_GAIN);
    adc_pid_1.set_sensor_bias(1,ADC_1_2_BIAS);
    adc_pid_1.set_PID_gains(1, ADC_1_2_KP, ADC_1_2_KI, ADC_1_2_KD);
    adc_pid_1.set_PID_setpoint(1, 0);
    // 3
    adc_pid_1.set_sensor_gain(2,ADC_1_3_GAIN);
    adc_pid_1.set_sensor_bias(2,ADC_1_3_BIAS);
    adc_pid_1.set_PID_gains(2, ADC_1_3_KP, ADC_1_3_KI, ADC_1_3_KD);
    adc_pid_1.set_PID_setpoint(2, 0);
    // 4
    adc_pid_1.set_sensor_gain(3,ADC_1_4_GAIN);
    adc_pid_1.set_sensor_bias(3,ADC_1_4_BIAS);
    adc_pid_1.set_PID_gains(3, ADC_1_4_KP, ADC_1_4_KI, ADC_1_4_KD);
    adc_pid_1.set_PID_setpoint(3, 0);
  }
  #endif
}
// update PIDs
inline void update(){
  #ifdef SPI_1_NUM
  spi_pid_1.update();
  #endif
  #ifdef SPI_2_NUM
  spi_pid_2.update();
  #endif
  #ifdef ACTUATOR_ARM_LENGTHS
  adc_pid_1.update();
  #endif
}
#endif
