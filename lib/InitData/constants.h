#ifndef CONST_H
#define CONST_H

const float WHEEL_RADIUS_LEFT = 0.04465;    // Радиус колеса левого [m]
const float WHEEL_RADIUS_RIGHT = 0.04465;   // Радиус колеса правого [m]
const float BASE_LENGTH = 0.285;            // Ширина базы между колёсами [m]

const float MAX_MOTOR_VELOCITY = 150.0;     // Максимальная скорость вращения двигателей [rmp]
const float MAX_LIN_SPEED = 0.7;            // Максимальная скорость робота [m/s]
const float MAX_LIN_ACCEL = 15;             // Максимальное линейное ускорение [m/s^2]
const float MAX_ANG_ACCEL = 65;             // Максимальное угловое ускорение [rad/s^2]
const float MIN_PWM = 20;                   // Минимальное услилие мотора (0-255)

const int DELAY = 40;                       // частота работы алгоритма управления движением [ms] (чаще чем 23-25 ms ArduinoMega2560 не может)
const int DELAY_MOTOR = 60;                 // частота обновления сокрости мотора [ms]
const int DELAY_MSG = 1000;                 // частота отправки сообщений [ms]

// Коэффициенты для контроллера управления дифферинциальным приводом по кривизне траеткории
const float DIFF_K1 = 1;            // Настроечный коэффициент для расчёта K
const float DIFF_K2 = 2;            // Настроечный коэффициент для расчёта K 
const float DIFF_K3 = 0.6;          // Настроечный коэффициент для расчёта v и w
const float DIFF_K4 = 1;            // Настроечный коэффициент для расчёта w при вращении на месте
//const float DIFF_K4 = 0.835;      // Настроечный коэффициент для расчёта w при вращении на месте
const float DIFF_K_MAX = 2.5;       // Пороговая величина кривизны траектории

// Коэффициенты PID регулятора для контроллера управления движением по PID
const float PID_Kp = 2;
const float PID_Ki = 0.01;
const float PID_Kd = 0.000001; 

// Коэффициенты PID регулятора для левого и правого колёс
const float KpL = 0.8;
const float KiL = 0;
const float KdL = 0;
const float KpR = 0.83;
const float KiR = 0;
const float KdR = 0;

const float ALLOW_POS_ERR = 0.03;    // Доступная погрешность расстояния [m]
const float ALLOW_ANG_ERR = 0.0175;   // Доступная погрешность угла [rad]
const float ALLOW_FIN_ERR = 30;     // Итоговая доступная погрешность

// Номера пинов на I2С мультиплексоре для энкодеров
const int ENCODER_PIN_R = 2;
const int ENCODER_PIN_L = 7;

// ====================================
// ==== Пины на драйвере | Пины на МК =
// ====================================
const int DRIVER_PWM_PIN_A = 7;
const int DRIVER_IN_A2 = 6;
const int DRIVER_IN_A1 = 5;
const int DRIVER_IN_B1 = 4;
const int DRIVER_IN_B2 = 3;
const int DRIVER_PWM_PIN_B = 2;
// ====================================

const int RST_PIN = 8;          // Configurable, see typical pin layout above
const int SS_PIN = 53;

#endif // CONST_H