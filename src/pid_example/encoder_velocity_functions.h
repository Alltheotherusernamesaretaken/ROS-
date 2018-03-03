#ifndef ENCODER_VELOCITY_FUNCTIONS_H
#define ENCODER_VELOCITY_FUNCTIONS_H

#include <Arduino.h>

#define A_PIN 34
#define B_PIN 35

// Encoder variables
extern const uint32_t COUNTS_PER_MOTOR_REV; // 12 paddles on the encoder
extern const uint32_t EVENTS_PER_MOTOR_REV; // 4 logic changes
extern const uint32_t EVENTS_PER_GEARBOX_REV; // Gearing
extern portMUX_TYPE enc_mux;
extern volatile int32_t enc_count;
// This one is special; it's defined in pid_functions.cpp
extern double velocity;

// Encoder ISR Declarations
void A_int();
void B_int();
void update_vel();
void setup_encs();

#endif
