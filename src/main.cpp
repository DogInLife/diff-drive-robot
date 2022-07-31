#include <Arduino.h>
#include "twoWheeledRobot.h"
#include "constants.h"
#include "RFIDReader.h"


byte del = 50; // задержка
int whl_vel_des = 60; // скорость колеса [об/мин]
bool deb = false; // флаг типа дебаггинга

//float R = 0.5;

float KpL = 0.0;
float KiL = 0.0;
float KdL = 0.0;
float KpR = 0.0;
float KiR = 0.0;
float KdR = 0.0;

//TwoWheeledRobot robot;

void setup() {

  Serial.begin(38400);
  TwoWheeledRobot robot;
  robot.createWheels(WHEEL_RADIUS, BASE_LENGTH, MAX_VELOCITY);
  robot.setEncoderPins(ENCODER_PIN_L, ENCODER_PIN_R);
  robot.setDriverPins(DRIVER_PWM_PIN_A, DRIVER_IN_A2, DRIVER_IN_A1 , DRIVER_IN_B1, DRIVER_IN_B2,  DRIVER_PWM_PIN_B);
  //robot.tunePID(5.3, 4.8, 0);
  //robot.tunePID(20.0, 2.8, 0.5); // работает для движения в положение x y (без учёта угла, скорость - квадрат омеги в знаменателе) minrange 0
  //robot.tunePID(20.0, 2.8, 0.5); // ОНО САМОЕ ПРИЕЗЖАЕТ НА БАЗУ ДОПУСТИМОЕ ОТКЛОНЕНИЕ ОТ ТОЧКИ 0.03 скорость 3/10 от макс
  robot.tunePID(50.0, 2.0, 0.5);


  //robot.tunePID(3.4, 1.2, 0.9);
// ====== Д Л Я  120 ОБ/МИН ======
//   KpL = 600.0;
//   KiL = 12000.0;
//   KdL = 0.5;
// // ================  П Р О В Е Р Ь  ============
// // ================     Ф Л А Г     ============
// // ================      D E B      ============
//   KpR = 600.0;
//   KiR = 12000.0;
//   KdR = 0.5;

// ====== Д Л Я 60  ОБ/МИН =====
  KpL = 250.0;
  KiL = 5000.0;
  KdL = 0.5;
// ================  П Р О В Е Р Ь  ============
// ================     Ф Л А Г     ============
// ================      D E B      ============
  KpR = 250.0;
  KiR = 5000.0;
  KdR = 0.5;
  robot.tuneWhlPID(KpL, KiL, KdL, KpR, KiR, KdR);

  robot.serialControl(deb);
  //robot.goCircle(1.0, 8);
  //robot.manualControl(del);
  //robot.rot_test(whl_vel_des, del, deb); // ########################
}


void loop() {

}