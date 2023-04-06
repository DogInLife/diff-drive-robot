#ifndef VELOCITY_H
#define VELOCITY_H
#include <math.h>

class Velocity
{
private:
    float maxWheelRpm = 0;  // Макс. скорость вращения колёс [rpm]
    float maxWheel = 0;     // Макс. скорость вращения колёс [rad/s]
    float maxVelocity = 0;  // Макс. скорость робота [m/s]
    
public:
    float lin = 0;      // Линейная скорость робота [m/s]
    float ang = 0;      // Угловая скорость робота [rad/s]
    float wheelL = 0;   // Угловая скорость левого колеса [rmp]
    float wheelR = 0;   // Угловая скорость правого колеса  [rmp]

    /*
        maxWheelRpm - Макс. скорость вращения колёс [rpm]
        wheelR - Средний радиус колёс [m]
    */
    Velocity(float maxWheelRpm, float wheelR);
    ~Velocity();

    //================= SET =================//
    /*
        Устанавливает максимальную скорость вращения колёс [rpm]
        maxWheelRpm - Макс. скорость вращения колёс [rpm]
        wheelR - Средний радиус колёс [m]
    */
    void setMaxWheelRpm(float maxWheelRpm, float wheelR);

    //================= FUNCTIONS =================//
    /* Конвертирует угловую скорость из [rmp] в [rad/s]. */
    float rmp2rads(float rmp);
    /* Конвертирует угловую скорость из [rad/s] в [rmp]. */
    float rads2rmp(float rads);
    /* Возвращает линейную скорость.
       IN: vel - угловая скорость [m/s]
           wheelR - радиус колеса [m]
       OUT: линейная скорость [m/s] */
    float angular2linear(float ang, float wheelR);
    /* Возвращает угловую скорость.
       IN: vel - линейная скорость [m/s]
           wheelR - радиус колеса [m]
       OUT: угловая скорость [m/s] */
    float linear2angular(float vel, float wheelR);    
    
    //================= GET =================//
    /*
        Возвращает макс. скорость вращения колёс [rpm]
    */
    float getMaxWheelRpm();
    /*
        Возвращает макс. скорость вращения колёс [rad/s]
    */
    float getMaxWheel();
    /*
        Возвращает макс. скорость робота [m/s]
    */
    float getMaxVelocity();
};

#endif // VELOCITY_H