#include <Arduino.h>
#include "twoWheeledRobot.h"
#include "constants.h"

byte del = 50; // задержка
int whl_vel_des = 60; // скорость колеса [об/мин]
bool deb = true; // флаг типа дебаггинга

void setup() {
  
  TwoWheeledRobot robot;
  robot.createWheels(WHEEL_RADIUS, BASE_LENGTH, MAX_VELOCITY);
  robot.setEncoderPins(ENCODER_PIN_L, ENCODER_PIN_R);
  robot.setDriverPins(DRIVER_PWM_PIN_A, DRIVER_IN_A2, DRIVER_IN_A1 , DRIVER_IN_B1, DRIVER_IN_B2,  DRIVER_PWM_PIN_B);
  // robot.tunePID(5.3, 4.8, 0);

  // ========== П Р О В Е Р Ь   Ф Л А Г   D E B   !!! =============
  robot.tunePID(20.0, 0.0, 0.1, /*_*/ 25.0, 0.0, 0.1); // (pL, iL, dL, pR, iR, dR)

  // robot.serialControl();
  // robot.manualControl(dt);
  robot.rot_test(whl_vel_des, del, deb);

  // float xGoal = 1;
  // float yGoal = 1;
  // robot.goToGoal(xGoal, yGoal, dt);
  // robot.manualControl();
}


void loop() {}
