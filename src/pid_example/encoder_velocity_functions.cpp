
#include "encoder_velocity_functions.h"

// Encoder variables
const uint32_t COUNTS_PER_MOTOR_REV = 12; // 12 paddles on the encoder
const uint32_t EVENTS_PER_MOTOR_REV = COUNTS_PER_MOTOR_REV*4; // 4 logic changes
const uint32_t EVENTS_PER_GEARBOX_REV = EVENTS_PER_MOTOR_REV*64; // Gearing
portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t enc_count=0;


void setup_encs(){
  // Interrupt pins
  pinMode(A_PIN, INPUT);
  pinMode(B_PIN, INPUT);

  // Encoder interrupts
  attachInterrupt(A_PIN, A_int, CHANGE);
  attachInterrupt(B_PIN, B_int, CHANGE);
}

// Update motor velocity
void update_vel(){
  static uint32_t prev_time = millis()-1;
  static uint32_t this_time;

  static int32_t prev_count = 0;
  static int32_t this_count;

  static double vel_avg[5] = {0};
  static uint32_t i = 0;

  this_time = millis();

  portENTER_CRITICAL(&enc_mux);
  this_count = enc_count;
  portEXIT_CRITICAL(&enc_mux);

  // rad/s = events/duration / EVENTS_PER_GEARBOX_REV * 2*pi
  vel_avg[i] = (1000 * (double) (this_count-prev_count)) / (double) (this_time-prev_time) / EVENTS_PER_GEARBOX_REV * 2 * 3.14 / 5;
  velocity = vel_avg[0] + vel_avg[1] + vel_avg[2] + vel_avg[3] + vel_avg[4];
  i = (i+1)%5;
  // update prev_time to this_time
  prev_time = this_time;
  // Could optionally clear enc_count, but this way allows positioning wrt start
  prev_count = this_count;
}

// Encoder ISR's
// Need special parameters to place function in quick-access RAM
// See: https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/
void IRAM_ATTR A_int(){
  //enc_count++;

  static uint32_t A_bit, B_bit, xor_bit;
  // Read A channel
  #if A_PIN < 32
    A_bit = (GPIO.in >> A_PIN) & 0x1;
  #else
    A_bit = (GPIO.in1.val >> (A_PIN - 32)) & 0x1;
  #endif
  #if B_PIN < 32
    B_bit = (GPIO.in >> B_PIN) & 0x1;
  #else
    B_bit = (GPIO.in1.val >> (B_PIN - 32)) & 0x1;
  #endif

  xor_bit = A_bit ^ B_bit;

  switch(xor_bit){
    case(1):
      portENTER_CRITICAL_ISR(&enc_mux);
      enc_count++;
      portEXIT_CRITICAL_ISR(&enc_mux);
      break;
    case(0):
      portENTER_CRITICAL_ISR(&enc_mux);
      enc_count--;
      portEXIT_CRITICAL_ISR(&enc_mux);
      break;
  }//*/
}

void IRAM_ATTR B_int(){
  //enc_count++;

  static uint32_t A_bit, B_bit, xor_bit;
  // Read A channel
  #if A_PIN < 32
    A_bit = (GPIO.in >> A_PIN) & 0x1;
  #else
    A_bit = (GPIO.in1.val >> (A_PIN - 32)) & 0x1;
  #endif
  #if B_PIN < 32
    B_bit = (GPIO.in >> B_PIN) & 0x1;
  #else
    B_bit = (GPIO.in1.val >> (B_PIN - 32)) & 0x1;
  #endif

  xor_bit = A_bit ^ B_bit;

  switch(xor_bit){
    case(1):
      portENTER_CRITICAL_ISR(&enc_mux);
      enc_count--;
      portEXIT_CRITICAL_ISR(&enc_mux);
      break;
    case(0):
      portENTER_CRITICAL_ISR(&enc_mux);
      enc_count++;
      portEXIT_CRITICAL_ISR(&enc_mux);
      break;
  }//*/
}
