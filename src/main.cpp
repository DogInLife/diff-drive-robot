#include <Arduino.h>
#include "twoWheeledRobot.h"
#include "constants.h"
#include "RFIDReader.h"

void setup()
{
    Serial.begin(9600);
    TwoWheeledRobot robot;
    robot.serialControl(DELAY, DELAY_MSG, false);

    // *старые комментарии
    // robot.tunePID(5.3, 4.8, 0);
    // robot.tunePID(20.0, 2.8, 0.5); // работает для движения в положение x y (без учёта угла, скорость - квадрат омеги в знаменателе) minrange 0
    // robot.tunePID(20.0, 2.8, 0.5); // ОНО САМОЕ ПРИЕЗЖАЕТ НА БАЗУ ДОПУСТИМОЕ ОТКЛОНЕНИЕ ОТ ТОЧКИ 0.03 скорость 3/10 от макс
    // robot.tunePID(12.0, 5.0, 0.3);
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
    /*
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
    */
}

void loop()
{
    /*int analogValue = analogRead(A0);     // Считываем аналоговое значение
    Serial.println((String)analogValue);
    delay(100);*/
}
