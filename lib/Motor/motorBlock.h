#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "encoder.h"
#include "pid.h"
#include "constants.h"
/*
    Класс для управления мотором по скорости с обратной связью по энкодеру.
    Для обработки измерения поворота угла и скорости необходимо вызывать tick()!
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

    PID* speedPID;                  // PID для управления скоростью колеса
    float Kp = KpL, Ki = KiL, Kd = KdL;   // PID коэффициенты       
    float del = 40;                 // Время дискретизации [ms]

    float angleOffset = 0;          // Угол поворота колеса за время после предыдущего измерения [rad]
    unsigned long lastTime = 0;     // Последнее время измерения [ms]
    float speedSum = 0;             // Сумма скоростей для вычисления средней [rad/s]
    int speedCount = 0;             // Число скоростей для вычисления средней
    float lastSpeed = 0;            // Последняя вычисленная скорость [rad/s]
    float targetSpeed = 0;          // Заданная скорость [rad/s]

    /*
        Обновляет усилие на моторе по PID регулятору на ошибке скорости.
        dt - время дискретизации [ms]
    */
    void updatePWM(float dt);
    /*
        Возвращает скорость вращения колеса [rad/s]
    */
    float getWheelSpeed();
    /*
        Возвращает угол поворота колеса за время после предыдущего измерения для измерения сокрости [rad]
    */
    float getDeltaAngle();

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
        Устанавливает коэффициенты PID регулятора.
        Kp, Ki, Kd - коэффициенты
    */
    void setCoefficientPID(float Kp, float Ki, float Kd);
    /*
        Устанавливает частоту дискретизации для PID регулятора.
        dt - время дискретизации [ms]
    */
    void setDeltaPID(float dt);
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
        Выполняет подготовку таймеров.
    */
    void tickPrepare();
    /*
        Функция для обработки прерывания. Перед началом работы рекоммендуется вызвать функцию tickPrepare();
        Выполняет рассчёт угола поворота колеса за время после предыдущего измерения.
        Выполняет рассчёт скорости вращения.
    */
    void tick();
    /*
        Обнуляет накопленные ошибки.
    */
    void resetErrPID();
    /*
        Останавливает движение.
    */
    void stopMoving();
    /*
        Обновляет скорость.
        vel - скорость вращения [rad/s]
    */
    void updateVelocity(float vel);
    /*
        Обновляет усилие на моторе по PID регулятору на ошибке скорости.
        Время дискретизации = del [ms]
    */
    void updatePWM();

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
        Возвращает угол поворота колеса за время после предыдущего измерения [rad]
    */
    float getAngleOffset();
    /*
        Возвращает последнюю рассчитанную скорость в Tick [rad/s]
    */
    float getLastSpeed();

};
#endif // MOTOR_H
