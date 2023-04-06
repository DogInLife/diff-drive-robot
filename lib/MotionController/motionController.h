#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H
#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include "diffDriveController.h"

class MotionController {
    private:
        DiffDriveController* driveController;
        
        float available_pos_err = 0.005;    // Доступная погрешность расстояния [m]
        float available_ang_err = 0.0175;   // Доступная погрешность угла [rad]
        float available_reach_err = 30;     // Итоговая доступная погрешность

        float delta = 0;        // Угол между направлением на целевую точку и текущим направлением движения робота [rad] 
        float distance = 0;     // Расстояние между текущим положением робота и целевой позицией [m]
        float theta = 0;        // Угол  между направлением на целевую точку и вектором скорости робота в целевой точке [rad]

        float v = 0;            // Линейная скорость [m/s]
        float w = 0;            // Угловая скорость [rad/s]
        float wheelL = 0;       // Скорость левого колеса [rad/s]
        float wheelR = 0;       // Скорость правого колеса [rad/s]

    public:
        float x = 0, y = 0;     // Текущие координаты робота [m]
        float alpha = 0;        // Текущее направление робота [m]
        float _x = 0, _y = 0;   // Конечные координаты робота [m]
        float beta = 0;         // Конечное направление робота [m]
        bool isUsedBeta = false;    // true, если учитывается угол Beta

        MotionController();
        /*
            d - ширина базы между колёсами [m]
            rL, rR - радиусы колёс [m]
            v_max - максимальная линейная скорость [m/s]
        */
        MotionController(float d, float rL, float rR, float v_max);
        ~MotionController();

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
            k1, k2, k3, k4 - настроечные коэффициенты
            K_max - пороговая величина кривизны траектории
        */
        void setCoefficient(float k1, float k2, float k3, float k4, float K_max);

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
        void updateVelocity();      // !!! Можно добавить ввод рельных скоростей колёс, чтобы контроллер в себе смягчил ускорение
        /*
            Возвращает наименьший угол со знаком между двумя векторами. Знак определяется тем, в какую сторону открывается второй вектор относительно первого. Например, при открытии по часовой стрелке знак будет положительным.
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
            Возвращает кривизну траектории движения робота
        */
        float get_K();
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