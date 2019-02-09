#include "MotorInterfaceABC.h"
#include <map>
#include <string>
#include <vector>

class MotorManager: MotorInterfaceABC
{
protected:
    /// \brief A vector containing the MotorInterface objects
    std::vector<MotorInterfaceABC*> motors;
    /// \brief A map that connects human-readable motor names to the appropriate motor index
    std::map<std::string, int> motor_map;
    /// \brief A mutex for threadsafing motor access
    SemaphoreHandle_t mtx;

    /// \brief This function accesses the map and returns the appropriate id
    int getMotorIdx(std::string, int*);
    /// \brief This function writes a value to the motor index
    void writeMotorPos(double,int);
    void writeMotorVel(double,int);

    void start(){ xSemaphoreTake( mtx, portMAX_DELAY );}
    void end(){ xSemaphoreGive( mtx );}

public:
    
    MotorManager(std::vector<MotorInterfaceABC*> motors, std::map<std::string, int> motor_map);

    /// \brief This function sets a motor position setpoint via a string identifier
    int setPos(double, std::string);
    /// \brief This function sets motor position setpoints via string identifiers
    int setPos(double*, std::string*, int);
    
    int getPos(std::string, double*);
    int getPos(std::string*, int, double*);
    
    int setVel(double, std::string);
    int setVel(double*, std::string*, int);
    
    int getVel(std::string, double*);
    int getVel(std::string*, int, double*);

    // Overloaded methods
    int update();

    int setPos(double, int);
    int setVel(double, int);

    int getPos(double*,int);
    int getVel(double*,int);

};