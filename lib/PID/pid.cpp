#include "pid.h"

PID::PID() 
: Kp(0.0), Ki(0.0), Kd(0.0),
  errOld(0.0), errSum(0.0), errDot(0.0)
{}

PID::~PID(){}

float PID::computeControl(float err, float dt)
{
    errDot = err - errOld; // дельта соседних измерений
    errSum = errSum + err; // суммарная ошибка за всё время

    float u = Kp*err + Ki*errSum*dt + Kd*errDot/dt;
    errOld = err;
    return u;
}

float PID::computeAngleError(float thetaGoal, float theta)
{
    float err = thetaGoal - theta;
    if(err > 3.141593) return err - 2*3.141593;
    else if(err < -3.141593) return err + 2*3.141593;
    else return err;
}

float PID::computeLineError(float sens_1, float sens_2)
{
    float err = (sens_1 - sens_2)*3.141593;
    return err;
}

void PID::setCoefficient(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::resetErr()
{
    this->errOld = 0.0;
    this->errSum = 0.0;
    this->errDot = 0.0;
}