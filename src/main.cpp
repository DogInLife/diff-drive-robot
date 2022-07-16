#include <Arduino.h>
#include "twoWheeledRobot.h"
#include "constants.h"

byte del = 50; // задержка
int whl_vel_des = 60; // скорость колеса [об/мин]
bool deb = false; // флаг типа дебаггинга

float KpL;
float KiL;
float KdL;
float KpR;
float KiR;
float KdR;

void setup() {
  
  TwoWheeledRobot robot;
  robot.createWheels(WHEEL_RADIUS, BASE_LENGTH, MAX_VELOCITY);
  robot.setEncoderPins(ENCODER_PIN_L, ENCODER_PIN_R);
  robot.setDriverPins(DRIVER_PWM_PIN_A, DRIVER_IN_A2, DRIVER_IN_A1 , DRIVER_IN_B1, DRIVER_IN_B2,  DRIVER_PWM_PIN_B);
  // robot.tunePID(5.3, 4.8, 0);

  KpL = 20.0;
  KiL = 0.0;
  KdL = 0.05;
// ================  П Р О В Е Р Ь  ============
// ================     Ф Л А Г     ============
// ================      D E B      ============
  KpR = 25.0;
  KiR = 0.0;
  KdR = 0.05;
  
  robot.tunePID(KpL, KiL, KdL, KpR, KiR, KdR);
  //robot.tunePID(20.0, 0.0, 0.05, /*_*/ 25.0, 0.0, 0.05); // (pL, iL, dL, pR, iR, dR)

  // robot.serialControl();
  // robot.manualControl(dt);
  robot.rot_test(whl_vel_des, del, deb);

  // float xGoal = 1;
  // float yGoal = 1;
  // robot.goToGoal(xGoal, yGoal, dt);
  // robot.manualControl();
}


void loop() {}
