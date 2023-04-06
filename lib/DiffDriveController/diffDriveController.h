#ifndef DIFF_DRIVE_CONTROLLER_H
#define DIFF_DRIVE_CONTROLLER_H
#include <Arduino.h>
#include <math.h>
#include <SPI.h>

class DiffDriveController {
    private:
        float d = 0.285;        // Ширина базы между колёсами [m]
        float rL = 0.04465;     // Радиус левого колeса [m]
        float rR = rL;          // Радиус правого колёса [m]
        float v_max = 0.7014;   // Максимальная линейная скорость [m/s]
        float k1 = 3, k2 = 10, k3 = 0.35, k4 = 1.1425;   // Настроечные коэффициенты
        float K_max = 30;       // Пороговая величина кривизны траектории

        float K = 0;            // Кривизна траектории движения робота в данной точке
        float delta = 0;        // Угол между направлением на целевую точку и текущим направлением движения робота [rad] 
        float distance = 0;     // Расстояние между текущим положением робота и целевой позицией [m]
        float theta = 0;        // Угол  между направлением на целевую точку и вектором скорости робота в целевой точке [rad]

        float v = 0;            // Линейная скорость [m/s]
        float w = 0;            // Угловая скорость [rad/s]
        float wheelL = 0;       // Скорость левого колеса [rad/s]
        float wheelR = 0;       // Скорость правого колеса [rad/s]

        void calculateTrajectoryCurvature();    // Раcчёт K
        void calculateVelocity();               // Раcчёт v и w
        void calculateWheelSpeed();             // Раcчёт wheelL и wheelR

    public:
        float _x = 0, _y = 0;   // Конечные координаты робота [m]

        DiffDriveController();
        /*
            d - ширина базы между колёсами [m]
            rL, rR - радиусы колёс [m]
            v_max - максимальная линейная скорость [m/s]
        */
        DiffDriveController(float d, float rL, float rR, float v_max);
        ~DiffDriveController();

        //================= SET =================//
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
        
        //================= FUNCTIONS =================//
        /*
            Обновляет значения скоростей для достижения целевой координаты.
            delta - угол между направлением на целевую точку и текущим направлением движения робота [rad] 
            distance - расстояние между текущим положением робота и целевой позицией [m]
            theta - угол  между направлением на целевую точку и вектором скорости робота в целевой точке [rad]
        */
        void updateVelocity(float delta, float distance, float theta);
        
        //================= GET =================//
        /* 
            Возвращает кривизну траектории движения робота
        */
        float get_K();
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