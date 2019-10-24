#include "pid.h"

Pid::Pid(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    pid = 0;
    error_last = 0;
    error_sum = 0;
}

void Pid::set(double target)
{
    this->target = target;
}

double Pid::compute(double current, double dt)
{
    error = target - current;
    // pid = kp * error + ki * error_sum * dt + kd * (error - error_last) / dt;
    pid = kp * error + ki * error_sum * dt + kd * (error - error_last) / dt;
    error_last = error;
    error_sum += error;

    // Integral Reset when reaches 0
    if (target == 0 && error == 0)
    {
        pid = 0;
        error_last = 0;
        error_sum = 0;
    }

    return pid > 1 ? 1 : pid < -1 ? -1 : pid;
}