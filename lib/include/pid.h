#ifndef _PID
#define _PID

class Pid
{
public:
    Pid(double kp, double ki, double kd);
    void set(double target);
    double compute(double current, double dt);
    double kp, ki, kd, error, error_last, error_sum, target, pid;
};

#endif