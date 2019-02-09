#include <Arduino.h>
class MotorInterfaceABC
{
protected:
    bool pos_ctrl=false;
    double pos_set=0;
    double vel_set=0;
    double pos_upper_limit=-1e7;
    double pos_lower_limit=1e7;
    double vel_upper_limit=-1e7;
    double vel_lower_limit=1e7;
public:
    MotorInterfaceABC(){};
    ~MotorInterfaceABC(){};

    /// \brief Updates motor with new setpoints, etc.
    virtual int update() = 0;

    /// \brief Sets the new Position setpoint
    ///
    /// \param setpoint The new setpoint value
    /// \param idx Optional channel index
    virtual int setPos(double setpoint, int idx=0){
        pos_set = setpoint;
    }

    /// \brief Sets the new Velocity setpoint
    ///
    /// \param setpoint The new setpoint value
    /// \param idx Optional channel index
    virtual int setVel(double setpoint, int idx=0){
        vel_set = setpoint;
    }

    /// \brief Gets Position setpoint
    ///
    /// \param setpoints Pointer to hold setpoint
    /// \param idx Optional channel index
    virtual int getPos(double* setpoint, int idx=0);

    /// \brief Gets Velocity setpoint
    ///
    /// \param setpoints Pointer to hold setpoint
    /// \param idx Optional channel index
    virtual int getVel(double* setpoint, int idx=0);

    /// \brief Sets the Position limits
    ///
    /// \param lower Lower limit
    /// \param upper Upper limit
    /// \param idx Optional channel index
    virtual int setPosLimits(double lower=-1e7, double upper=1e7, int idx=0){
        if (lower>upper) return 1;
        if (lower>-1e7) pos_lower_limit = lower;
        if (upper<1e7) pos_upper_limit = upper;
    }

    /// \brief Sets the Velocity limits
    ///
    /// \param lower Lower limit
    /// \param upper Upper limit
    /// \param idx Optional channel index
    virtual int setVelLimits(double lower=-1e7, double upper=1e7, int idx=0){
        if (lower>upper) return 1;
        if (lower>-1e7) vel_lower_limit = lower;
        if (upper<1e7) vel_upper_limit = upper;
    }

    /// \brief Gets the Position limits
    ///
    /// \param lower Lower limit pointer
    /// \param upper Upper limit pointer
    /// \param idx Optional channel index
    virtual int getPosLimits( double* lower, double* upper, int idx=0){
        if (lower!=NULL) *lower = pos_lower_limit;
        if (upper!=NULL) *upper = pos_upper_limit;
    }

    /// \brief Gets the Velocity limits
    ///
    /// \param lower Lower limit pointer
    /// \param upper Upper limit pointer
    /// \param idx Optional channel index
    virtual int getVelLimits( double* lower, double* upper, int idx=0){
        if (lower!=NULL) *lower = vel_lower_limit;
        if (upper!=NULL) *upper = vel_upper_limit;
    }

    /// \brief Sets the control type (Pos/Vel)
    ///
    /// \param pos_ctrl Control type (True/False) (Pos/Vel)
    virtual int setPosCtrl(bool posctrl, int=0){
        pos_ctrl = posctrl;
    }

    /// \brief Gets the control type (Pos/Vel)
    ///
    /// \param pos_ctrl Pointer for control type (True/False) (Pos/Vel)
    virtual int getPosCtrl(bool* posctrl, int=0){
        *posctrl = pos_ctrl;
    }

};
