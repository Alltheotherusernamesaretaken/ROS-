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
            arg[0][i][0] = (void*) &(muxes[i]); // mux
            arg[0][i][1] = (void*) &(A[i]); // A
            arg[0][i][2] = (void*) &(B[i]); // B
            arg[0][i][3] = (void*) &(B[i % 2]); // flip
            arg[0][i][4] = (void*) &(counts[i]); // count
            attachInterruptArg(A[i], &ISR, (void*) arg[0][i], CHANGE);
            // B interrupts
            arg[1][i][0] = (void*) &(muxes[i]); // mux
            arg[1][i][1] = (void*) &(A[i]); // A
            arg[1][i][2] = (void*) &(B[i]); // B
            arg[1][i][3] = (void*) &(B[1-(i % 2)]); // flip
            arg[1][i][4] = (void*) &(counts[i]); // count
            attachInterruptArg(B[i], &ISR, (void*) arg[0][i], CHANGE);
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
        return 0
    }

    int PID_update()
    {
        xSemaphoreTake(mtx, portMAX_DELAY);
        for(int i=0; i<4; i++)
        {
            PIDs[i]->Compute();
        }
        xSemaphoreGive(mtx);
        // write PWM
        for(int i=0; i<4; i++)
        {
            // TODO: 
        }
    }

    // pins
    int pwm[4];
    int dir[4];
    int A[4];
    int B[4];
    bool flip[2] = {false, true};

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

    // encoder ISR stuff
    void* arg[2][4][5]; // ugly, but wasn't sure if they'd stay in scope if local to INT_init
    int32_t counts[4] = {0};
    portMUX_TYPE muxes[4] = {portMUX_INITIALIZER_UNLOCKED};

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
            A[i] = INT_A_PINS[i];
            B[i] = INT_B_PINS[i];
        }
        xSemaphoreGive(mtx);
        
    }
    ~Quad_PWM(){}

    typedef struct
    {
        typedef struct
        {
            portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
            uint8_t A;
            uint8_t B;
            int32_t count;
            bool flip;
        } channel;
        channel A, B;

    } encoder;
    

    // hardware parameters
    double cpr, wheel_rad, base_width;

    // r/w mutex
    static SemaphoreHandle_t mtx;

    void init()
    {
        PIN_init();
        INT_init();
        PID_init();
    }





    // static method for ISR
    // This is NUTS but oughtta work
    static void IRAM_ATTR ISR(void*){
    /* Inputs is void* to a void* array of the following:
     * portMUX_TYPE* - mux for isr blocking
     * int*          - Encoder Pin A
     * int*          - Encoder Pin B
     * bool*         - Boolean to flip increment direction
     * int32_t*     - count to increment
     */
    static void** arr;
    static int enc;
    arr = static_cast<void**>(arg);
    portMUX_TYPE* mux = static_cast<portMUX_TYPE*>(arr[0]);
    portENTER_CRITICAL_ISR(mux);

    enc = digitalRead(*(int*)(arr[1])) ^ digitalRead(*(int*)(arr[2])) ^ (*(bool*)arr[3]);
    switch (enc)
    {
    case 1:
        (*(int32_t*)arr[4])++;
        break;
    
    case 0:
        (*(int32_t*)arr[4])--;
        break;
    }

    portEXIT_CRITICAL_ISR(mux);
}

};


