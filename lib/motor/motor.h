

class Motor
{
public:
    Motor(){}
    ~Motor(){}
    
    virtual int init(){return 0;}

    virtual int update(){return 0;}

    virtual int set_twist(double lin, double ang){return 0;}

    virtual int get_twist(double *lin, double *ang){return 0;}
};