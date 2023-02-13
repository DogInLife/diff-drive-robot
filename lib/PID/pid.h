#ifndef PID_H
#define PID_H
#include "math.h"

class PID
{
    
private:
    float Kp;
    float Ki;
    float Kd;
    float errOld;
    float errSum;
    float errDot;

public: 
    PID();
    ~PID();
    
    float computeControl(float err, float dt);
    float computeAngleError(float thetaGoal, float theta);
    float computeLineError(float sens_1, float sens_2);
    void setCoefficient(float Kp, float Ki, float Kd);
    void resetErr();
};

#endif //PID_H