#include "position.h"

Position::Position() {}

void Position::computeCurentPose(float D_L, float D_R, float D_C, float L)
{   
    float cos_th = 0.0;
    float sin_th = 0.0;
    if (alpha == M_PI/2)
        cos_th = 0.0;
    else
        cos_th = cos(alpha);
    
    if (alpha == 0.0)
        sin_th = 0.0;
    else
        sin_th = sin(alpha);

    x = D_C * cos_th;   
    y = D_C * sin_th;
    alpha = (D_R-D_L)/L;
}

// оценка текущей позиции робота по данным с энкодеров
void Position::estCurrentPosition(float deltaAng_L, float deltaAng_R, float r, float L) {
    //float avgXerr = 0.01; // средняя погрешность измерений по X на метр пройденного пути
    //float avgYerr = -0.047; // средняя погрешность измерений по Y на метр пройденного пути
    
    // float avgXerr = 0.04872 - 0.01369;
    // float avgYerr = -0.03613 + 0.00816;

    // float avgXerr = 0.02399;
    // float avgYerr = -0.00405;

    // float dPath = 0.1; // дельта расстояния, за которое учитывается погрешность измерений

    float cos_th = 0.0;
    float sin_th = 0.0;

    if(alpha == M_PI/2)
        cos_th = 0.0;
    else
        cos_th = cos(alpha);
    
    if(alpha == 0.0)
        sin_th = 0.0;
    else
        sin_th = sin(alpha);

    // перемещения вдоль осей в собственной СК робота
    float dXR;
    float dYR;

    float fric = 0.99;

    // перемещения вдоль осей в глобольной СК
    float deltaX;
    float deltaY;

    float deltaAlpha = r * (deltaAng_R - deltaAng_L) * fric / L;
    float curveR;
    bool infCurveR = false;

    if(fabs(deltaAlpha) < 0.01)
        infCurveR = true;
        //curveR = 777; // )))0

    else curveR = (L * (deltaAng_R + deltaAng_L)) / (2.0 * (deltaAng_R - deltaAng_L)); 

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
        if(deltaAlpha == 3.141593/2)
            cos_dth = 0.0;
        else
            cos_dth = cos(deltaAlpha);
    
        if(deltaAlpha == 0.0)
            sin_dth = 0.0;
        else
            sin_dth = sin(deltaAlpha);

        // изменение координат в собственной СК
        dXR = curveR * sin_dth;
        dYR = curveR * (1 - cos_dth);

        // изменение координат в глобальной СК
        deltaX = dXR * cos_th - dYR * sin_th;
        deltaY = dXR * sin_th + dYR * cos_th;
    }

    x = x + deltaX;
    y = y + deltaY;
    
    float nextAlpha = alpha + deltaAlpha;
    if(nextAlpha > M_PI) alpha = nextAlpha - 2*M_PI;
    else if(nextAlpha < -M_PI) alpha = nextAlpha + 2*M_PI;
    else alpha = nextAlpha; 
}

void Position::correctPosEst(float distWheelC) {
    float avgXerr = 0.02503;
    float avgYerr = -0.00585;

    float dPath = 0.1;

    if(distWheelC-distWheelPrev >= dPath) {
        x = x + (dPath * avgXerr);
        y = y + (dPath * avgYerr);
        distWheelPrev = distWheelC;
        corrected = true;
    }
}