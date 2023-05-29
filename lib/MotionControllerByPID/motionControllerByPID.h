#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H
#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include "constants.h"
#include "diffDriveControllerByPID.h"
/*
    Контроллер управления движением. Для расчёта желаемых скоростей используется контроллер дифферинциального привода на основе PID регулятора (diffDriveControllerByPID).

    Возможности:
    - Расчёт угола между направлением на целевую точку и текущим направлением движения робота
    - Расчёт расстояния между текущим положением робота и целевой позицией
    - Оценка достигаемости целевой позиции
    - Расчёт желаемых скоростей робота (линейная и угловая)
    - Расчёт желаемых скоростей колёс дифферинциального привода
*/
class MotionControllerByPID {
    private:
        DiffDriveControllerByPID* driveController;
        
        float available_pos_err = ALLOW_POS_ERR;    // Доступная погрешность расстояния [m]

        float delta = 0;        // Угол между направлением на целевую точку и текущим направлением движения робота [rad] 
        float distance = 0;     // Расстояние между текущим положением робота и целевой позицией [m]
        //float theta = 0;        // Угол  между направлением на целевую точку и вектором скорости робота в целевой точке [rad]

        float v = 0;            // Линейная скорость [m/s]
        float w = 0;            // Угловая скорость [rad/s]
        float wheelL = 0;       // Скорость левого колеса [rad/s]
        float wheelR = 0;       // Скорость правого колеса [rad/s]

        float del = DELAY;      // Частота [ms]

    public:
        float x = 0, y = 0;     // Текущие координаты робота [m]
        float alpha = 0;        // Текущее направление робота [m]
        float _x = 0, _y = 0;   // Конечные координаты робота [m]
        float beta = 0;         // Конечное направление робота [m]
        bool isUsedBeta = false;    // true, если учитывается угол Beta

        MotionControllerByPID();
        /*
            d - ширина базы между колёсами [m]
            rL, rR - радиусы колёс [m]
            v_max - максимальная линейная скорость [m/s]
        */
        MotionControllerByPID(float d, float rL, float rR, float v_max);
        ~MotionControllerByPID();

        //================= SET PARAMETERS =================//
        /*
            Устанавливает характеристики робота.
            d - ширина базы между колёсами [m]
            rL, rR - радиусы колёс [m]
            v_max - максимальная линейная скорость [m/s]
        */
        void setRobotConstant(float d, float rL, float rR, float v_max);
        /*
            Устанавливает коэффициенты.
            Kp, Ki, Kd - коэффициенты PID регулятора
        */
        void setCoefficient(float Kp, float Ki, float Kd);
        /*
            Устанавливает задержку
            del - задержка [ms]
        */
        void setDelay(float del);

        //================= SET USING VALUES =================//
        /*
            Задаёт текущее положение робота.
            x, y - текущие координаты [m]
            alpha - текущее направление [rad]
        */
        void setActualPosition(float x, float y, float alpha);
        /*
            Задаёт конечное положение робота.
            _x, _y - конечные координаты [m]
        */
        void setTargetPosition(float _x, float _y);
        /*
            Задаёт конечное положение робота.
            _x, _y - конечные координаты [m]
            beta - конечное направление [rad]
        */
        void setTargetPosition(float _x, float _y, float beta);
        /*
            Задаёт флаг использования конечного направления beta. Если true, то угол учитывается.
        */
        void setBetaIsUsed(bool isUsedBeta);

        //================= FUNCTIONS =================//
        /*
            Обновляет значения скоростей, необходимых для достижения целевой координаты.
        */
        void updateVelocity();
        /*
            Возвращает наименьший угол со знаком между двумя векторами. Знак определяется тем, в какую сторону открывается второй вектор относительно первого.
            a_x, a_y - проекции первого вектора A на оси координат [m]
            b_x, b_y - проекции второго вектора B на оси координат [m]
        */
        float angleBetweenVectorsWithSign(float a_x, float a_y, float b_x, float b_y);
        /*
            Возвращает True, если позиция достигнута, иначе False.
        */
        bool isReachPosition();
        
        //================= GET =================//
        /* 
            Возвращает угол между направлением на целевую точку и текущим направлением движения робота [rad] 
        */
        float get_delta();
        /* 
            Возвращает расстояние между текущим положением робота и целевой позицией [m]
        */
        float get_distance();
        /* 
            Возвращает угол между направлением на целевую точку и вектором скорости робота в целевой точке [rad]
        */
        float get_theta();
        /* 
            Возвращает рассчитанную линейную скорость [m/s]
        */
        float get_v();
        /* 
            Возвращает рассчитанную угловую скорость [rad/s]
        */
        float get_w();
        /* 
            Возвращает рассчитанную скорость левого колеса [rad/s]
        */
        float get_wheelL();
        /* 
            Возвращает рассчитанную скорость правого колеса [rad/s]
        */
        float get_wheelR();
};
#endif