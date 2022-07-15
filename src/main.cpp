#include <Arduino.h>
#include "twoWheeledRobot.h"
#include "constants.h"

byte del = 25; // задержка
int whl_vel = 30; // скорость колеса [об/мин]

void setup() {
  
  TwoWheeledRobot robot;
  robot.createWheels(WHEEL_RADIUS, BASE_LENGTH, MAX_VELOCITY);
  robot.setEncoderPins(ENCODER_PIN_L, ENCODER_PIN_R);
  robot.setDriverPins(DRIVER_PWM_PIN_A, DRIVER_IN_A2, DRIVER_IN_A1 , DRIVER_IN_B1, DRIVER_IN_B2,  DRIVER_PWM_PIN_B);
  // robot.tunePID(5.3, 4.8, 0);
  robot.tunePID(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // robot.serialControl();
  // robot.manualControl(dt);
  robot.rot_test(whl_vel, del);

  // float xGoal = 1;
  // float yGoal = 1;
  // robot.goToGoal(xGoal, yGoal, dt);
  // robot.manualControl();
}


void loop() {}
