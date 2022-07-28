#ifndef CONST_H
#define CONST_H

#define DEBUG  0
#define DEBUG_PLOT 0

#define WHEEL_RADIUS 0.0443  //m
#define BASE_LENGTH 0.285   //m

#define MAX_VELOCITY 150  // об/мин у двигателей

// Номера пинов на I2С мультиплексоре для энкодеров
#define ENCODER_PIN_R 2
#define ENCODER_PIN_L 7

// ====================================
// ==== Пины на драйвере | Пины на МК =
// ====================================
#define DRIVER_PWM_PIN_A    7
#define DRIVER_IN_A2        6
#define DRIVER_IN_A1        5
#define DRIVER_IN_B1        4
#define DRIVER_IN_B2        3 
#define DRIVER_PWM_PIN_B    2
// ====================================

#endif // CONST_H