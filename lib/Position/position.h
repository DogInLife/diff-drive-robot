#ifndef POSITION_H
#define POSITION_H
#include "math.h"

class Position{
private:
    
public:
    float x;
    float y;
    float theta;

    float xGoal;
    float yGoal;
    float thetaGoal;

    Position();
    void computeCurentPose(float D_L, float D_R, float D_C, float L);
    void estCurrentPosition(float deltaAng_L, float deltaAng_R, float r, float L); // оценка текущей позиции робота по данным с энкодеров
{
};


#endif // POSITION_H