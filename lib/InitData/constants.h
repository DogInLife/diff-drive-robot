#ifndef CONST_H
#define CONST_H

#define DEBUG  0
#define DEBUG_PLOT 0

const float WHEEL_RADIUS_LEFT = 0.04465;    // Радиус колеса левого [m]
const float WHEEL_RADIUS_RIGHT = 0.04465;   // Радиус колеса правого [m]
const float BASE_LENGTH = 0.285;            // Ширина базы между колёсами [m]

const float MAX_MOTOR_VELOCITY = 150.0;     // Максимальная скорость вращения двигателей [об/мин]

// Коэффициенты PID регулятора для левого и правого колёс
const float KpL = 30.0;
const float KiL = 0.05;
const float KdL = 0.01;
const float KpR = 30.0;
const float KiR = 0.05;
const float KdR = 0.01;

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