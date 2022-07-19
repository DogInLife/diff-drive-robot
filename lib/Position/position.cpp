#include "position.h"


Position::Position() 
:   x(0.0), y(0.0), theta(0.0),
    xGoal(0.0), yGoal(0.0), thetaGoal(0.0)
{}

void Position::computeCurentPose(float D_L, float D_R, float D_C, float L)
{   
    float cos_th = 0.0;
    float sin_th = 0.0;
    if (theta == 3.141593/2)
        cos_th = 0.0;
    else
        cos_th = cos(theta);
    
    if (theta == 0.0)
        sin_th = 0.0;
    else
        sin_th = sin(theta);


    x = D_C * cos_th;   
    y = D_C * sin_th;
    theta = (D_R-D_L)/L;
    //thetaGoal = atan2(yGoal-y, xGoal-x);

}

// оценка текущей позиции робота по данным с энкодеров
void Position::estCurrentPosition(float deltaAng_L, float deltaAng_R, float r, float L)
{

    float cos_th = 0.0;
    float sin_th = 0.0;

    if(theta == 3.141593/2)
        cos_th = 0.0;
    else
        cos_th = cos(theta);
    
    if(theta == 0.0)
        sin_th = 0.0;
    else
        sin_th = sin(theta);

    // перемещения вдоль осей в собственной СК робота
    float dXR;
    float dYR;

    // перемещения вдоль осей в глобольной СК
    float deltaX;
    float deltaY;

    float deltaTheta = r * (deltaAng_R - deltaAng_L) / L;
    float curveR;
    bool infCurveR = false;

    if(fabs(deltaTheta) < 0.01)
        infCurveR = true;
        //curveR = 777; // )))0

    else curveR = (L * (deltaAng_R + deltaAng_L)) / (2.0 * (deltaAng_R - deltaAng_L)); // ОБРАБОТАТЬ ДЕЛЕНИЕ НА НОЛЬ

    float cos_dth = 0.0;
    float sin_dth = 0.0;

    if(infCurveR)
    {
        cos_dth = 1.0;
        sin_dth = 0.0;
        
        deltaX = ((deltaAng_R + deltaAng_R) * r / 2.0) * cos_th;
        deltaY = ((deltaAng_R + deltaAng_L) * r / 2.0) * sin_th;
    } 
    
    else
    {
        if(deltaTheta == 3.141593/2)
            cos_dth = 0.0;
        else
            cos_dth = cos(deltaTheta);
    
        if(deltaTheta == 0.0)
            sin_dth = 0.0;
        else
            sin_dth = sin(deltaTheta);

        // изменение координат в собственной СК
        dXR = curveR * sin_dth;
        dYR = curveR * (1 - cos_dth);

        // изменение координат в глобальной СК
        deltaX = dXR * cos_th - dYR * sin_th;
        deltaY = dXR * sin_th + dYR * cos_th;
    }

    x = x + deltaX;
    y = y + deltaY;
    
    float nextTheta = theta + deltaTheta;
    if(newTheta > 3.141593) theta = nextTheta - 2*3.141593;
    else if(newTheta < -3.141593) theta = newTheta + 2*3.141593;
    else theta = newTheta; 

}