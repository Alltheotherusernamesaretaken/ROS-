#include "MotorInterfaceABC.h"

class MotorInterfaceTest: public MotorInterfaceABC
{
public:
    MotorInterfaceTest(){};

    int update(){return 0;}

    int getPos(double* setpoint, int idx=0){
        *setpoint = pos_set;
        return 0;
    }
    int getVel(double* setpoint, int idx=0){
        *setpoint = vel_set;
        return 0;
    }
};