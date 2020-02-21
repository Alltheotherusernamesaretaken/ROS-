#include <Arduino.h>
#include "motor.h"
#include <PID_v1.h>
// parameter externs
// PWM pins
extern const int FL_PWM;
extern const int FR_PWM;
extern const int BL_PWM;
extern const int BR_PWM;
// DIR pins
extern const int L_DIR_1;
extern const int L_DIR_2;
extern const int R_DIR_1;
extern const int R_DIR_2;
// PWM range
extern const int PWM_MAX;

// encoder counts per revolution
extern const int cpr;


class Quad_PWM: public Motor
{
    typedef struct
    {
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        uint8_t A;
        uint8_t B;
        int32_t count;
        bool dir;
    } encoder;

protected:
    void PID_init()
    {
        for(int i = 0; i<4; i++){
            PIDs[i]->SetOutputLimits(-1*pwm_max, pwm_max);
            PIDs[i]->SetSampleTime(50);
            PIDs[i]->SetMode(AUTOMATIC);
        }
    }

    void PIN_init()
    {
        for(int i=0; i<4; i++){
            // direction enable pins
            pinMode(dir[i], OUTPUT);
            // interrupt pins
            pinMode(A[i], INPUT);
            pinMode(B[i], INPUT);
            // PWM pins
            ledcSetup(i, 20000, 8);
            ledcAttachPin(pwm[i], i);
            // write defaults
            digitalWrite(dir[i], LOW);
            ledcWrite(i, 0);
        }
        vTaskDelay(1/portTICK_PERIOD_MS);
    }

    void INT_init()
    {
        for(int i = 0; i<4; i++)
        {
            // A interrupts
            attachInterruptArg(encs[i].A, &ISR_A, (void*) &(encs[i]), CHANGE);
            // B interrupts
            attachInterruptArg(encs[i].B, &ISR_B, (void*) &(encs[i]), CHANGE);
        }
    }

    int vel_update()
    {
        static unsigned long start=0, stop;
        static double delta_t = 0;
        static int32_t cnt[4];
        static int32_t prev_cnt[4];
        static double cpr_to_meter = 2*PI*wheel_rad/cpr; // meters per pulse

        stop = millis();
        delta_t = ((double) (stop - start)) / 1000.0;
        start = stop;
        for(int i=0; i<4; i++)
        {
            portENTER_CRITICAL(&muxes[i]);
            cnt[i] = counts[i];
            portEXIT_CRITICAL(&muxes[i]);
        }
        // split into two loops to reduce time of port mux and mutex usage
        xSemaphoreTake(mtx, portMAX_DELAY);
        for(int i=0; i<4; i++)
        {
            vels[i] = ((double) (cnt[i] - prev_cnt[i])) / delta_t * cpr_to_meter;
            prev_cnt[i] = cnt[i];
        }
        xSemaphoreGive(mtx);
        return 0;
    }

    int PID_update()
    {
        xSemaphoreTake(mtx, portMAX_DELAY);
        for(int i=0; i<4; i++)
        {
            PIDs[i]->Compute();
        }
        xSemaphoreGive(mtx);
        // left side
        digitalWrite(dir[0], (effs[0] > 0) ? HIGH : LOW);
        digitalWrite(dir[1], (effs[0] > 0) ? LOW : HIGH);
        // right side
        digitalWrite(dir[2], (effs[2] > 0) ? HIGH : LOW);
        digitalWrite(dir[3], (effs[2] > 0) ? LOW : HIGH);
        // write PWM
        for (int i=0; i<4; i++)
        {
            ledcWrite(i, (uint32_t) round(abs(effs[0])));
        }
    }

    // pins
    int pwm[4];
    int dir[4];

    encoder encs[4];
    // PID setpoint, input, output
    double sets[4], vels[4], effs[4];
    // pwm max
    int pwm_max;

    // PID objects
    PID FL_PID;
    PID FR_PID;
    PID BL_PID;
    PID BR_PID;
    // access pids via this
    PID* PIDs[4];
    // hardware parameters
    double cpr, wheel_rad, base_width;
    // r/w mutex
    static SemaphoreHandle_t mtx;

public:
    Quad_PWM(
        int* PWM_PINS,
        int* DIR_PINS,
        int* INT_A_PINS,
        int* INT_B_PINS,
        int PWM_MAX,
        double cpr, double wheel_rad, double base_width,
        double KP, double KI, double KD 
    ):
        FL_PID(&(vels[0]), &(effs[0]), &(sets[0]), KP, KI, KD, DIRECT),
        FR_PID(&(vels[0]), &(effs[0]), &(sets[0]), KP, KI, KD, DIRECT),
        BL_PID(&(vels[0]), &(effs[0]), &(sets[0]), KP, KI, KD, DIRECT),
        BR_PID(&(vels[0]), &(effs[0]), &(sets[0]), KP, KI, KD, DIRECT),
        cpr(cpr),
        wheel_rad(wheel_rad),
        base_width(base_width),
        pwm_max(PWM_MAX)
    {
        mtx = xSemaphoreCreateMutex();
        xSemaphoreTake(mtx, portMAX_DELAY);
        // set PID references now that PID's are created
        // kinda wonky but keeps PID's on the stack
        // really important to minimize heap on embedded systems
        PIDs[0] = &FL_PID;
        PIDs[1] = &FR_PID;
        PIDs[2] = &BL_PID;
        PIDs[3] = &BR_PID;

        // store pins
        for(int i=0; i<4; i++){
            pwm[i] = PWM_PINS[i];
            dir[i] = DIR_PINS[i];
            encs[i].A = INT_A_PINS[i];
            encs[i].B = INT_B_PINS[i];
            encs[i].count = 0;
            encs[i].dir = i%2;
        }
        xSemaphoreGive(mtx);
        
    }
    ~Quad_PWM(){}

    int init()
    {
        PIN_init();
        INT_init();
        PID_init();
        return 0;
    }

    int update()
    {
        vel_update();
        PID_update();
        return 0;
    }

    int set_twist(double lin, double ang)
    {
        // convert lin/ang to l/r vels
        if (abs(ang) > 0.01)
        {
            double r = abs(lin/ang);
            double angmag = abs(ang);
            double sgnlin = lin > 0 ? 1 : -1;
            double sgnang = ang > 0 ? 1 : -1;

            sets[0] = sgnlin*angmag*(r - sgnlin*sgnang*base_width/2.0);
            sets[2] = sgnlin*angmag*(r - sgnlin*sgnang*base_width/2.0);

            sets[1] = sgnlin*angmag*(r + sgnlin*sgnang*base_width/2.0);
            sets[3] = sgnlin*angmag*(r + sgnlin*sgnang*base_width/2.0);
        } else // easy, just lin vel
        {
            sets[0] = lin;
            sets[1] = lin;
            sets[2] = lin;
            sets[3] = lin;
        }
    }

    int get_twist(double *lin, double *ang)
    {
        *lin = (vels[0]+vels[1]+vels[2]+vels[3]) / 4.0;
        *ang = (vels[1]+vels[3]-vels[0]-vels[2])/base_width / 2.0;
        return 0;
    }

    // static method for ISR
    // This is NUTS but oughtta work
    static void IRAM_ATTR ISR_A(void* arg){
    /* Inputs is void* to a void* array of the following:
     * portMUX_TYPE* - mux for isr blocking
     * int*          - Encoder Pin A
     * int*          - Encoder Pin B
     * bool*         - Boolean to flip increment direction
     * int32_t*     - count to increment
     */
    static encoder* enc;
    static int dir;
    enc = static_cast<encoder*>(arg);
    portENTER_CRITICAL_ISR(&(enc->mux));
    dir = digitalRead(enc->A) ^ digitalRead(enc->B) ^ enc->dir;
    switch (dir)
    {
    case 1:
        enc->count++;
        break;
    
    case 0:
        enc->count--;
        break;
    }
    portEXIT_CRITICAL_ISR(&(enc->mux));
}
    static void IRAM_ATTR ISR_B(void* arg){
    /* Inputs is void* to a void* array of the following:
     * portMUX_TYPE* - mux for isr blocking
     * int*          - Encoder Pin A
     * int*          - Encoder Pin B
     * bool*         - Boolean to flip increment direction
     * int32_t*     - count to increment
     */
    static encoder* enc;
    static int dir;
    enc = static_cast<encoder*>(arg);
    portENTER_CRITICAL_ISR(&(enc->mux));
    dir = digitalRead(enc->A) ^ digitalRead(enc->B) ^ enc->dir;
    switch (dir)
    {
    case 1:
        enc->count--;
        break;
    
    case 0:
        enc->count++;
        break;
    }
    portEXIT_CRITICAL_ISR(&(enc->mux));
}
};


