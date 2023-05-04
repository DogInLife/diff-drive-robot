#ifndef PID_H
#define PID_H
#include "math.h"
/*
    Пропорционально-интегрально-дифференцирующий (ПИД) регулятор
*/
class PID
{
private:
    float Kp;
    float Ki;
    float Kd;

    float errOld;
    float errSum;
    float errDot;

    float errResult;

public: 
    PID(float Kp, float Ki, float Kd);
    ~PID();

    void setCoefficient(float Kp, float Ki, float Kd);
    float updateErr(float err, float dt);
    float getErr();
    void resetErr();
};

#endif