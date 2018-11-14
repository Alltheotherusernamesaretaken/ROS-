
class MotorInterfaceABC
{
private:
    bool pos_ctrl=false;
    int num_motors=1;
    double pos_set=0;
    double vel_set=0;
    double pos_upper_limit=-1e7;
    double pos_lower_limit=1e7;
    double vel_upper_limit=-1e7;
    double vel_lower_limit=1e7;
public:
    MotorInterfaceABC();
    ~MotorInterfaceABC();

    /// \brief Updates motor with new setpoints, etc.
    virtual int update();

    /// \brief Sets the new Position setpoint
    ///
    /// \param setpoint The new setpoint value
    /// \param idx Optional channel index
    virtual int setPos(double setpoint, int idx=0);
    /// \brief Sets the new Velocity setpoint
    ///
    /// \param setpoint The new setpoint value
    /// \param idx Optional channel index
    virtual int setVel(double setpoint, int idx=0);
    /// \brief Sets the new Position setpoints for multiple motors
    ///
    /// \param setpoints The array of new setpoints
    /// \param indexes The array of indexes
    /// \param idx Optional channel index
    virtual int setPos(double* setpoints, int* indexes, int idx=0);
    /// \brief Sets the new Velocity setpoints for multiple motors
    ///
    /// \param setpoints The array of new setpoints
    /// \param indexes The array of indexes
    /// \param idx Optional channel index
    virtual int setVel(double* setpoints, int* indexes, int idx=0);
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
    /// \brief Gets the Position setpoints for multiple motors
    ///
    /// \param setpoints The array to hold setpoints
    /// \param indexes The array of indexes
    /// \param idx Optional channel index
    virtual int getPos(double* setpoints, int* indexes, int=0);
    /// \brief Gets the Velocity setpoints for multiple motors
    ///
    /// \param setpoints The array to hold setpoints
    /// \param indexes The array of indexes
    /// \param idx Optional channel index
    virtual int getVel(double* setpoints, int* indexes, int=0);
    /// \brief Sets the Position limits
    ///
    /// \param lower Lower limit
    /// \param upper Upper limit
    /// \param idx Optional channel index
    virtual int setPosLimits(double lower=-1e7, double=1e7, int=0);
    /// \brief Sets the Velocity limits
    ///
    /// \param lower Lower limit
    /// \param upper Upper limit
    /// \param idx Optional channel index
    virtual int setVelLimits(double lower=-1e7, double=1e7, int=0);
    /// \brief Gets the Position limits
    ///
    /// \param lower Lower limit pointer
    /// \param upper Upper limit pointer
    /// \param idx Optional channel index
    virtual int getPosLimits( double* lower, double* upper, int=0);
    /// \brief Gets the Velocity limits
    ///
    /// \param lower Lower limit pointer
    /// \param upper Upper limit pointer
    /// \param idx Optional channel index
    virtual int getVelLimits( double* lower, double* upper, int=0);
    /// \brief Sets the control type (Pos/Vel)
    ///
    /// \param pos_ctrl Control type (True/False) (Pos/Vel)
    virtual int setPosCtrl(bool, int=0);
    /// \brief Gets the control type (Pos/Vel)
    ///
    /// \param pos_ctrl Pointer for control type (True/False) (Pos/Vel)
    virtual int getPosCtrl(bool*, int=0);

};
