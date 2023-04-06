#include "twoWheeledRobot.h"

TwoWheeledRobot::TwoWheeledRobot()
{
    if (!Serial)
        Serial.begin(9600);

    wheelRadius = (WHEEL_RADIUS_LEFT + WHEEL_RADIUS_RIGHT) / 2.0;
    baseLength = BASE_LENGTH;
    vel = new Velocity(MAX_MOTOR_VELOCITY, wheelRadius);
    pos = new Position();

    motorBlockL = new MotorBlock(WHEEL_RADIUS_LEFT, ENCODER_PIN_L, DRIVER_IN_B1, DRIVER_IN_B2, DRIVER_PWM_PIN_B);
    motorBlockR = new MotorBlock(WHEEL_RADIUS_RIGHT, ENCODER_PIN_R, DRIVER_IN_A1, DRIVER_IN_A2, DRIVER_PWM_PIN_A);

    pidL = new PID(KpL, KiL, KdL);
    pidR = new PID(KpR, KiR, KdR);

    motionControler = new MotionController();
}

TwoWheeledRobot::~TwoWheeledRobot()
{
    delete vel;
    delete pos;
    delete motorBlockL;
    delete motorBlockR;
    delete pidL;
    delete pidR;
    delete motionControler;
}

//================= GET =================//
byte TwoWheeledRobot::getSerialData()
{
    return Serial.read();
}

// === Control ===
void TwoWheeledRobot::serialControl(int del, bool deb)
{
    while (true)
    {
        if (globalStop)
        {
            stopMoving();
            Serial.println("========= Choose mode =========");
            Serial.println(" 1 = Manual control ");
            Serial.println(" 2 = Simple move");
            Serial.println(" 3 = CW trajectory ");
            Serial.println(" 4 = CCW trajectory ");
            Serial.println(" 5 = Go to NULL position (not yet)");
            Serial.println(" 6 = Go by magnetic line. Debug");
            Serial.println(" 7 = Go by magnetic line. Go to point");
            Serial.println(" press \"z\" to stop ");
            globalStop = false;
        }

        inByte = getSerialData();
        globalSerialControl(inByte);
        switch (inByte)
        {
        case ('1'):
            Serial.println("========= You are using manual control =========");
            manualControl(del);
            break;

        case ('2'):
            Serial.println("========= Simple move =========");

            break;

        case ('3'):
            Serial.println("========= CW trajectory =========");
            // goCWtest(1.0, del, deb);
            Serial.println("Coming soon");
            break;

        case ('4'):
            Serial.println("========= CCW trajectory =========");
            // goCCWtest(1.0, del, deb);
            Serial.println("Coming soon");
            break;

        case ('6'):
            Serial.println("========= Go by magnetic line. Debug =========");
            // resertPosition();
            // goMagneticLine(del, true);
            Serial.println("Coming soon");
            break;

        case ('7'):
            Serial.println("========= Go by magnetic line. Go to point =========");
            // resertPosition();
            // goMagneticLine2Point(3, -1, del, deb);
            // if (!globalStop) {
            //   stopMoving();
            //   globalStop = true;
            //   Serial.println("Fatal error");
            // }
            Serial.println("Coming soon");
            break;
        }
    }
}

void TwoWheeledRobot::globalSerialControl()
{
    globalSerialControl(getSerialData());
}

void TwoWheeledRobot::globalSerialControl(byte inB)
{
    switch (inB)
    {
        case ('z'):
            globalStop = true;
            stopMoving();
            break;
    }
}

// ====================== robot behavior ===================== //

void TwoWheeledRobot::manualControl(int del)
{
    int vel = 60;

    float r = wheelRadius;
    float L = baseLength;

    float deltaAngL;
    float deltaAngR;

    float distWheelL = 0.0;
    float distWheelR = 0.0;
    float distWheelC = 0.0;

    while (!globalStop)
    {
        inByte = getSerialData();
        globalSerialControl(inByte);
        switch (inByte)
        {
        case ('w'):
            goForward(vel, vel);
            break;
        case ('s'):
            goForward(-vel, -vel);
            break;
        case (' '):
            stopMoving();
            break;
        case ('d'):
            turnRight(vel, -vel);
            break;
        case ('a'):
            turnLeft(-vel, vel);
            break;
        case ('e'):
            vel += 5;
            (vel >= 150) ? 150 : vel;
            break;
        case ('q'):
            vel -= 5;
            (vel <= 0) ? 0 : vel;
            break;
        }

        deltaAngL = motorBlockL->getDeltaAngle();
        deltaAngR = motorBlockR->getDeltaAngle();

        distWheelL = distWheelL + deltaAngL * r;
        distWheelR = distWheelR + deltaAngR * r;
        distWheelC = (distWheelR + distWheelL) / 2;

        pos->estCurrentPosition(deltaAngL, deltaAngR, r, L);

        // Serial.println(distWheelC);

        // if(distWheelC >= 1.0) {
        //   Serial.println("1 m traveled, distance: " + String(distWheelC, 3));
        //   stopMoving();
        //   break;
        // }

        String msg_enc = String(pos->x, 3) + " " + String(pos->y, 3) + " " + String(pos->alpha);
        Serial.println(msg_enc);

        delay(del);
    }
}

void TwoWheeledRobot::resertPosition()
{
    pos->x = 0.0;
    pos->y = 0.0;
    pos->alpha = 0.0;
    pidL->resetErr();
    pidR->resetErr();
}

void TwoWheeledRobot::startGoToPosition(float _x, float _y, int del, bool deb)
{
    // задаём конечную позицию в контроллере
    motionControler->setBetaIsUsed(false);
    motionControler->setTargetPosition(_x, _y);

    goToPosition(del, deb);
}

void TwoWheeledRobot::startGoToPosition(float _x, float _y, float beta, int del, bool deb)
{
    // задаём конечную позицию в контроллере
    motionControler->setBetaIsUsed(true);
    motionControler->setTargetPosition(_x, _y, beta);

    goToPosition(del, deb);
}

void TwoWheeledRobot::goToPosition(int del, bool deb)
{
    // задаём текущую позицию в контроллере
    motionControler->setActualPosition(pos->x, pos->y, pos->alpha);

    // поворот колёс за время между оценкой положения робота
    float deltaAngL = 0.0;
    float deltaAngR = 0.0;

    while (!motionControler->isReachPosition())
    {
        globalSerialControl();
        if (globalStop)
            break;
            
        motionControler->updateVelocity();

        if (!deb)
            goForward(motionControler->get_wheelL(), motionControler->get_wheelR());

        deltaAngL = motorBlockL->getDeltaAngle();
        deltaAngR = motorBlockR->getDeltaAngle();

        pos->estCurrentPosition(deltaAngL, deltaAngR, wheelRadius, baseLength);
        String msg_pos = "X: " + String(pos->x, 3) + " Y: " + String(pos->y, 3) + " Th: " + String(pos->alpha, 3);
        Serial.println(msg_pos);

        delay(del);
    }

    Serial.println("TARGET POINT REACHED");
    Serial.println("X: " + String(pos->x) + " Y: " + String(pos->y));
    stopMoving();
}

void TwoWheeledRobot::goTrack(Position points[], int del, bool deb)
{
    size_t size = sizeof points / sizeof(Position);
    for (int i = 0; i < size; i++)
    {
        if (!globalStop)
            startGoToPosition(points[i].x, points[i].y, del, deb);
    }
}

void TwoWheeledRobot::goCWtest(float L, int del, bool deb)
{
    resertPosition();
    startGoToPosition(L, 0.0, del, deb);
    startGoToPosition(L, -L, del, deb);
    startGoToPosition(0.0, -L, del, deb);
    startGoToPosition(0.0, 0.0, del, deb);
    Serial.println("CW track finish");
}

void TwoWheeledRobot::goCCWtest(float L, int del, bool deb)
{
    resertPosition();
    startGoToPosition(L, 0.0, del, deb);
    startGoToPosition(L, L, del, deb);
    startGoToPosition(0.0, L, del, deb);
    startGoToPosition(0.0, 0.0, del, deb);
    Serial.println("CCW track finish");
}

// void TwoWheeledRobot::goMagneticLine(int del, bool deb) {
//   // поворот колёс за время между оценкой положения робота
//   float deltaAngL = 0.0;
//   float deltaAngR = 0.0;
//   // скорректированные скорости колёс
//   float velL;
//   float velR;

//   float r = wheelRadius;
//   float l = baseLength;
//   float err = 0.0;

//   // измерительный промежуток
//   //float dt = del / 1000.0;
//   float dt = del;

//   // Направление. Угол, расстояние до точки
//   float delta, distance;

//   magneticLineReader->updateAverageSignal();
//   size_t count = magneticLineReader->getSensorCount();
//   size_t middle_sen_i = count / 2 - 1;

//   String message = "";

//   while(!globalStop) {
//     globalSerialControl();

//     byte* checkResult = magneticLineReader->checkMagneticField();

//     byte leftDetect = 0;
//     for (size_t i = 0; i <= middle_sen_i; i++) {
//       leftDetect += checkResult[i];
//     }
//     byte rightDetect = 0;
//     for (size_t i = middle_sen_i+1; i < count; i++) {
//       rightDetect += checkResult[i];
//     }

//     // Движение прямо
//     delta = 0;
//     distance = magneticLineReader->hallSensors[middle_sen_i]->x;
//     if (leftDetect && rightDetect &&
//         ((leftDetect > rightDetect && leftDetect/rightDetect < 1.5) ||
//          (leftDetect < rightDetect && rightDetect/leftDetect < 1.5))) {
//       if (leftDetect+rightDetect < 4 && checkResult[middle_sen_i] && checkResult[middle_sen_i+1]) {
//         message = "Forward";
//       } else {
//         message = "Crossroads";
//         //stopMoving();
//         //globalStop = true;
//       }
//     }
//     else if (leftDetect && leftDetect > rightDetect) {
//       if (leftDetect == 1 && checkResult[middle_sen_i]) {
//         message = "Forward";
//       } else {
//         message = "Left to ";
//         for (size_t i = 0; i <= middle_sen_i; i++) {
//           if (checkResult[i]) {
//             message += String(i);
//             delta = atan(magneticLineReader->hallSensors[i]->y/magneticLineReader->hallSensors[i]->x);
//             distance = sqrt(pow(magneticLineReader->hallSensors[i]->x, 2) + pow(magneticLineReader->hallSensors[i]->y, 2));
//             break;
//           }
//         }
//       }
//     }
//     else if (rightDetect && rightDetect > leftDetect) {
//       if (rightDetect == 1 && checkResult[middle_sen_i+1]) {
//         message = "Forward";
//       } else {
//         message = "Right to ";
//         for (size_t i = count-1; i > middle_sen_i; i--) {
//           if (checkResult[i]) {
//             message += String(i);
//             delta = atan(magneticLineReader->hallSensors[i]->y/magneticLineReader->hallSensors[i]->x);
//             distance = sqrt(pow(magneticLineReader->hallSensors[i]->x, 2) + pow(magneticLineReader->hallSensors[i]->y, 2));
//             break;
//           }
//         }
//       }
//     }
//     else {
//       message = "NoLine";
//     }

//     motionControler->calculate(delta, distance, 0);

//     velL = (motionControler->get_v() - l*motionControler->get_w()/2)/r;
//     velR = (motionControler->get_v() + l*motionControler->get_w()/2)/r;
//     //Serial.println("get_v: " + String(motionControler->get_v()) + ". get_w: " + String(l*motionControler->get_w()/2));

//     if (deb)
//       Serial.println(message);
//     else
//       goForward(velL*20, velR*20);
//       //goForward(velL, velR);

//     deltaAngL = motorBlockL->getDeltaAngle();
//     deltaAngR = motorBlockR->getDeltaAngle();

//     pos.estCurrentPosition(deltaAngL, deltaAngR, r, l);

//     delay(del);
//   }
//   magneticLineReader->takeOffLed();
// }
// void TwoWheeledRobot::goMagneticLine2Point(float _x, float _y, int del, bool deb) {
//   // поворот колёс за время между оценкой положения робота
//   float deltaAngL = 0.0;
//   float deltaAngR = 0.0;
//   // скорректированные скорости колёс
//   float velL;
//   float velR;

//   float r = wheelRadius;
//   float l = baseLength;
//   float err = 0.0;

//   // измерительный промежуток
//   float dt = del;

//   // Направление. delta - угол, distance - расстояние до точки
//   float delta, distance;

//   size_t count = magneticLineReader->getSensorCount();
//   size_t middle_sen_i = count / 2 - 1;

//   String message = "";

//   int crossroadsCounter = 0;                          // Счётчик перекрёстков
//   float actualDistance = 0, coverDistance = 0, admissionDistance = 0.005;  // Дистанции для пересечения перекрёстка, mm
//   float actualAngle = 0, coverAngle = 0, admissionAngle = 0.06;  // Дистанции для пересечения перекрёстка, mm
//   bool isManeuvering = false;                         // Робот в процессе выполнения манёвра
//   int decision;                                       // Указание к действию

//   while(!globalStop) {
//     globalSerialControl();

//     if (isManeuvering) {  // Если выполняется манёвр
//       if (crossroadsCounter <= _x)
//         actualDistance = pos.x;
//       else
//         actualDistance = -pos.y;
//       if (coverDistance - actualDistance < admissionDistance) { // Если доехали до центра перекрёстка
//         if (decision == 1) {                          // Если манёвр "Движение вперёд"
//           isManeuvering = false;    // Движение вперёд окончено
//           message = "Finish crossroad: " + String(crossroadsCounter);
//         } else if (decision == 2 || decision == 3) {  // Если манёвр "Поворот"
//           actualAngle = pos.alpha;
//           if (abs(coverAngle - actualAngle) < admissionAngle) { // Если повернули
//             isManeuvering = false;  // Поворот окончен
//             message = "Finish crossroad: " + String(crossroadsCounter);
//           }
//           else {                                                // Иначе продолжаем поворачиваться
//             delta = coverAngle - actualAngle;
//             distance = 0;
//             message = "actualAngle " + String(actualAngle) + " | coverAngle " + String(coverAngle)  + " | delta " + String(delta);
//           }
//         } else {                                      // Иначе остановка
//           isManeuvering = false;
//           stopMoving();
//           globalStop = true;
//           message = "Finish. Last crossroad: " + String(crossroadsCounter);
//         }
//       }
//       else {                                                    // Иначе продолжаем движение прямо
//         delta = 0;
//         distance = coverDistance - actualDistance;
//         message = "actualDistance " + String(actualDistance) + " | coverDistance " + String(coverDistance);
//       }
//     }
//     else {                // Иначе обычное следование линии
//       // Опрос датчика магнитной линии
//       byte* checkResult = magneticLineReader->checkMagneticField();
//       byte leftDetect = 0;
//       for (size_t i = 0; i <= middle_sen_i; i++) {
//         leftDetect += checkResult[i];
//       }
//       byte rightDetect = 0;
//       for (size_t i = middle_sen_i+1; i < count; i++) {
//         rightDetect += checkResult[i];
//       }

//       /*  Если засекли линию слева и права
//           И линия не сильно смещена от центра,
//           то движемся по середине линии или засекли перекрёсток */
//       if (leftDetect && rightDetect &&
//           ((leftDetect > rightDetect && leftDetect/rightDetect < 2) ||
//           (leftDetect < rightDetect && rightDetect/leftDetect < 2))) {
//         if (leftDetect+rightDetect > 3) {   // Если площадь детектирования большая, то засекли перекрёсток
//           message = "Crossroad";
//           decision = moveDecision(_x, _y, ++crossroadsCounter);
//           // Дистанция до пересечения центра перекрёстка и центра робота
//           //actualDistance = sqrt(pos.x*pos.x + pos.y*pos.y);
//           if (crossroadsCounter <= _x)
//             actualDistance = pos.x;
//           else
//             actualDistance = -pos.y;
//           coverDistance = actualDistance + magneticLineReader->hallSensors[middle_sen_i]->x;
//           // Угол поворота на перекрёстке
//           if (decision == 0)
//             message += ". Finish";
//           else if (decision == 1)
//             message += ". Forward";
//           else if (decision == 2) {
//             actualAngle = pos.alpha;
//             coverAngle = actualAngle + PI/2;
//             message += ". Left";
//           } else if (decision == 3) {
//             actualAngle = pos.alpha;
//             coverAngle = actualAngle - PI/2;
//             message += ". Right";
//           }
//           // Начало манёвра
//           isManeuvering = true;
//           Serial.println(message);
//           continue;
//         }
//         else {                              // Продолжаем движение по центру линии
//           message = "Forward";
//           delta = 0.;
//           distance = magneticLineReader->hallSensors[middle_sen_i]->x;
//         }
//       } else if (leftDetect && leftDetect > rightDetect) {      // Если преобладает левое направление
//         if (leftDetect == 1 && checkResult[middle_sen_i]) {     // Если линия находится по центру, движемся прямо
//           message = "Forward";
//           delta = 0.;
//           distance = magneticLineReader->hallSensors[middle_sen_i]->x;
//         } else {                                                // Иначе поворот налево
//           message = "Left";
//           for (size_t i = 0; i <= middle_sen_i; i++) {
//             if (checkResult[i]) {
//               delta = atan(magneticLineReader->hallSensors[i]->y/magneticLineReader->hallSensors[i]->x);
//               distance = sqrt(pow(magneticLineReader->hallSensors[i]->x, 2) + pow(magneticLineReader->hallSensors[i]->y, 2));
//               break;
//             }
//           }
//         }
//       }
//       else if (rightDetect && rightDetect > leftDetect) {       // Если преобладает правое направление
//         if (rightDetect == 1 && checkResult[middle_sen_i+1]) {  // Если линия находится по центру, движемся прямо
//           message = "Forward";
//           delta = 0.;
//           distance = magneticLineReader->hallSensors[middle_sen_i]->x;
//         } else {                                                // Иначе поворот направо
//           message = "Right";
//           for (size_t i = count-1; i > middle_sen_i; i--) {
//             if (checkResult[i]) {
//               delta = atan(magneticLineReader->hallSensors[i]->y/magneticLineReader->hallSensors[i]->x);
//               distance = sqrt(pow(magneticLineReader->hallSensors[i]->x, 2) + pow(magneticLineReader->hallSensors[i]->y, 2));
//               break;
//             }
//           }
//         }
//       }
//       else {
//         message = "NoLine";
//         delta = 0.;
//         distance = magneticLineReader->hallSensors[middle_sen_i]->x;
//       }
//     }

//     motionControler->calculate(delta, distance, 0);

//     if (isManeuvering && distance == 0 && (decision == 2 || decision == 3)) {
//       velL = (motionControler->get_v() + l*motionControler->get_w()/2/r) * (decision == 2 ? -2 : 2);
//       velR = -velL;
//       Serial.println(String(motionControler->get_v()) + " " + String(l*motionControler->get_w()/2/r));
//     }
//     else {
//       velL = (motionControler->get_v() - l*motionControler->get_w()/2)/r;
//       velR = (motionControler->get_v() + l*motionControler->get_w()/2)/r;
//     }
//     //Serial.println("get_v: " + String(motionControler->get_v()) + ". get_w: " + String(l*motionControler->get_w()/2));

//     velL = linear2angular(velL);
//     velR = linear2angular(velR);

//     if (message.compareTo("NoLine")) {
//       //Serial.println(message);
//       Serial.println("velL: " + String(velL) + " | velR: " + String(velR) + (" (rpm)") );
//     }

//     if (!deb)
//       goForward(velL, velR);

//     deltaAngL = motorBlockL->getDeltaAngle();
//     deltaAngR = motorBlockR->getDeltaAngle();

//     pos.estCurrentPosition(deltaAngL, deltaAngR, r, l);

//     delay(del);
//   }
//   magneticLineReader->takeOffLed();
// }

void TwoWheeledRobot::stopMoving()
{
    motorBlockL->stopMoving();
    motorBlockR->stopMoving();
}

void TwoWheeledRobot::goForward(int velL, int velR)
{
    motorBlockL->updateVelocity(velL, vel->getMaxWheelRpm(), newMinRange);
    motorBlockR->updateVelocity(velR, vel->getMaxWheelRpm(), newMinRange);
}

void TwoWheeledRobot::turnLeft(int velL, int velR)
{
    motorBlockL->updateVelocity(velL, vel->getMaxWheelRpm(), newMinRange);
    motorBlockR->updateVelocity(velR, vel->getMaxWheelRpm(), newMinRange);
}

void TwoWheeledRobot::turnRight(int velL, int velR)
{
    motorBlockL->updateVelocity(velL, vel->getMaxWheelRpm(), newMinRange);
    motorBlockR->updateVelocity(velR, vel->getMaxWheelRpm(), newMinRange);
}