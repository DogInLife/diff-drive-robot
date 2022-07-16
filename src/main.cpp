#include <Arduino.h>
#include "twoWheeledRobot.h"
#include "constants.h"

byte del = 50; // задержка
int whl_vel_des = 30; // скорость колеса [об/мин]
bool deb = true; // флаг типа дебаггинга

float KpL = 0.0;
float KiL = 0.0;
float KdL = 0.0;
float KpR = 0.0;
float KiR = 0.0;
float KdR = 0.0;

void setup() {
  
  TwoWheeledRobot robot;
  robot.createWheels(WHEEL_RADIUS, BASE_LENGTH, MAX_VELOCITY);
  robot.setEncoderPins(ENCODER_PIN_L, ENCODER_PIN_R);
  robot.setDriverPins(DRIVER_PWM_PIN_A, DRIVER_IN_A2, DRIVER_IN_A1 , DRIVER_IN_B1, DRIVER_IN_B2,  DRIVER_PWM_PIN_B);
  // robot.tunePID(5.3, 4.8, 0);

  //KpL = 30.0;
  KiL = 1000.0;
  //KdL = 0.5;
// ================  П Р О В Е Р Ь  ============
// ================     Ф Л А Г     ============
// ================      D E B      ============
  //KpR = 35.0;
  KiR = 1000.0;
  //KdR = 0.5;
  
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
