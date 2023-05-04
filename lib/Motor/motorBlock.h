#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.h"
/*
    Класс для управления мотором по скорости с обратной связью по энкодеру.
*/
class MotorBlock
{
private:
    float wheelRadius;              // Радиус колeса [m]
    float maxVel;                   // Максимальная скорость вращения [rad/s]

    Encoder* encoder;               // Энкодер

    float distanceTraveled_k1;
    float distanceTraveled_k0;

    byte PWM_PIN;
    float pwm;
    
    byte IN_DRIVER_PIN_1;
    byte IN_DRIVER_PIN_2;

public:
    /*
        wheelRadius - радиус колеса [m]
        encPin - подключение энкодера
        driverPin1, driverPin2, driverPinPWM - подключение драйвера
    */
    MotorBlock(float wheelRadius, float maxVel, byte encPin, byte driverPin1, byte driverPin2, byte driverPinPWM);
    ~MotorBlock();

    //================= SET =================//
    /*
        Устанавливает радиус колеса.
        wheelRadius - радиус колеса [m]
    */
    void setWheelRadius(float wheelRadius);
    /*
        Устанавливает максимальную скорость вращения
        maxVel - скорость вращения [rad/s]
    */
    void setMaxVel(float maxVel);
    /*
        Устанавливает пины энкодера.
    */
    void setEncorerPin(byte encPin);
    /*
        Устанавливает пины драйвера.
    */
    void setDriverPin(byte driverPin1, byte driverPin2, byte driverPinPWM);

    //================= FUNCTIONS =================//
    /*
        Останавливает движение.
    */
    void stopMoving();
    /*
        Обновляет скорость.
        vel - скорость вращения [rad/s]
    */
    void updateVelocity(float vel);

    //================= GET =================//
    /*
        Возвращает пройденную дистанцию
    */
    float getTraveledDistance();
    /*
        Возвращает угол на который повернулось колесо
    */
    float getRotAngle();
    /*
        Возвращает угол поворота колеса за время между определением позиции робота
    */
    float getDeltaAngle();
    
};
#endif // MOTOR_H
