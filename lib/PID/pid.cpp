#include "pid.h"

PID::PID(float Kp, float Ki, float Kd) : errOld(0.0), errSum(0.0), errDot(0.0)
{
    setCoefficient(Kp, Ki, Kd);
}
PID::~PID(){}

void PID::setCoefficient(float Kp, float Ki, float Kd)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

float PID::updateErr(float err, float dt)
{
    errDot = err - errOld; // дельта соседних измерений
    errSum = errSum + err; // суммарная ошибка за всё время

    errResult = Kp*err + Ki*errSum*dt + Kd*errDot/dt;
    errOld = err;
    return errResult;
}

float PID::getErr() {
    return errResult;
}

void PID::resetErr()
{
    this->errOld = 0.0;
    this->errSum = 0.0;
    this->errDot = 0.0;
}