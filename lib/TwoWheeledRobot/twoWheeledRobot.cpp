#include "twoWheeledRobot.h"

TwoWheeledRobot::TwoWheeledRobot()
{
    if (!Serial)
        Serial.begin(9600);

    wheelRadius = (WHEEL_RADIUS_LEFT + WHEEL_RADIUS_RIGHT) / 2.0;
    baseLength = BASE_LENGTH;
    vel = new Velocity(MAX_MOTOR_VELOCITY, wheelRadius);
    pos = new Position();

    discretTimer = new TimerMs();
    motorTimer = new TimerMs();
    msgTimer = new TimerMs();
    
    // motorBlockL = new MotorBlock(WHEEL_RADIUS_LEFT, vel->linear2angular(MAX_LIN_SPEED, WHEEL_RADIUS_LEFT), ENCODER_PIN_L, DRIVER_IN_B1, DRIVER_IN_B2, DRIVER_PWM_PIN_B);
    // motorBlockR = new MotorBlock(WHEEL_RADIUS_RIGHT, vel->linear2angular(MAX_LIN_SPEED, WHEEL_RADIUS_RIGHT), ENCODER_PIN_R, DRIVER_IN_A1, DRIVER_IN_A2, DRIVER_PWM_PIN_A);
    // maxVel = MAX_MOTOR_VELOCITY - ограничение скорости вращения мотора (обусловлено характеристиками мотора)
    motorBlockL = new MotorBlock(WHEEL_RADIUS_LEFT, vel->getMaxWheel(), ENCODER_PIN_L, DRIVER_IN_B1, DRIVER_IN_B2, DRIVER_PWM_PIN_B);
    motorBlockR = new MotorBlock(WHEEL_RADIUS_RIGHT, vel->getMaxWheel(), ENCODER_PIN_R, DRIVER_IN_A1, DRIVER_IN_A2, DRIVER_PWM_PIN_A);
    motorBlockL->setCoefficientPID(KpL, KiL, KdL);
    motorBlockR->setCoefficientPID(KpR, KiR, KdR);
    // v_max вычисляется из MAX_LIN_SPEED - ограничение скорости движения робота (ниже чем возможная максимальная скорость вращения моторов)
    motionControler = new MotionController(baseLength, WHEEL_RADIUS_LEFT, WHEEL_RADIUS_RIGHT, vel->linear2angular(MAX_LIN_SPEED, WHEEL_RADIUS_LEFT));
    //motionControler = new MotionController(baseLength, WHEEL_RADIUS_LEFT, WHEEL_RADIUS_RIGHT, vel->getMaxWheel());
    //motionControler = new MotionControllerByPID(baseLength, WHEEL_RADIUS_LEFT, WHEEL_RADIUS_RIGHT, vel->getMaxWheel());
}

TwoWheeledRobot::~TwoWheeledRobot()
{
    delete vel;
    delete pos;
    delete discretTimer;
    delete msgTimer;
    delete motorBlockL;
    delete motorBlockR;
    delete motionControler;
}

//================= GET =================//
byte TwoWheeledRobot::getSerialData()
{
    return Serial.read();
}

// === Control ===
void TwoWheeledRobot::serialControl(int del, int del_motor, int del_msg, bool deb)
{
    //motionControler->setDelay(del);     // Раскомменитровать при использовании MotionControllerByPID !!!
    discretTimer->setTime(del);
    motorTimer->setTime(del_motor);
    motorBlockL->setDeltaPID(del_motor);
    motorBlockR->setDeltaPID(del_motor);
    msgTimer->setTime(del_msg);

    while (true)
    {
        if (globalStop)
        {
            stopMoving();
            Serial.println("========= Choose mode =========");
            Serial.println(" 0 = Reset position");
            Serial.println(" 1 = Manual control");
            Serial.println(" 2 = Simple move");
            Serial.println(" 3 = CW trajectory");
            Serial.println(" 4 = CCW trajectory");
            Serial.println(" 5 = Motor test");
            //Serial.println(" 5 = Go by magnetic line. Debug");
            //Serial.println(" 6 = Go by magnetic line. Go to point");
            Serial.println(" press \"z\" to stop");
            globalStop = false;
        }

        inByte = getSerialData();
        globalSerialControl(inByte);
        switch (inByte)
        {
            case ('0'):
            {
                Serial.println("========= Reset position =========");
                resertPosition();
                globalStop = true;
                break;
            }
            case ('1'):
            {
                Serial.println("========= You are using manual control =========");
                timersStart();
                manualControl(del);
                break;
            }
            case ('2'):
            {
            //     Serial.println("========= Simple move (x++) =========");
            //     timersStart();
            //     float start_x = pos->x;
            //     float finish_x = start_x + 1.0f;
            //     setPositionAndStartMove(finish_x, 0.0f, del, deb);
                Serial.println("========= Simple move (y++) =========");
                timersStart();
                float start_y = pos->y;
                float finish_y = start_y + 1.0f;
                setPositionAndStartMove(0.0f, finish_y, del, deb);
                globalStop = true;
                break;
            }
            case ('3'):
            {
                Serial.println("========= CW trajectory =========");    // По часовой стрелке
                timersStart();
                float len = 1.0f;
                setPositionAndStartMove(0.0f, len, del, deb);
                setPositionAndStartMove(len, len, del, deb);
                setPositionAndStartMove(len, 0.0f, del, deb);
                setPositionAndStartMove(0.0f, 0.0f, del, deb);
                globalStop = true;
                break;
            }
            case ('4'):
            {
                Serial.println("========= CCW trajectory =========");   // Против часовой стрелки
                timersStart();
                float len = 1.0f;
                setPositionAndStartMove(len, 0.0f, del, deb);
                setPositionAndStartMove(len, len, del, deb);
                setPositionAndStartMove(0.0f, len, del, deb);
                setPositionAndStartMove(0.0f, 0.0f, del, deb);
                globalStop = true;
                break;
            }
            case ('5'):
            {
                timersStart();
                resertMotorsPID();
                driveMoving(17, 17);

                float leftAngle = 0;
                float rightAngle = 0;
                float leftSpeed = 0;
                float rightSpeed = 0;

                while (!globalStop)
                {
                    timersTick();

                    // прерывание на консоль
                    globalSerialControl();

                    if (discretTimer->ready()) 
                    {
                    }
                    
                    if (msgTimer->ready()) 
                    {
                        leftSpeed = motorBlockL->getLastSpeed();
                        rightSpeed = motorBlockR->getLastSpeed();
                        leftAngle = motorBlockL->getAngleOffset();
                        rightAngle = motorBlockR->getAngleOffset();
                        // Serial.println("Angle: L " + String(leftAngle, 3) + 
                        //             " | R " + String(rightAngle, 3));
                        Serial.println("Angular [rad/s]: L " + String(leftSpeed, 3) + 
                                    " | R " + String(rightSpeed, 3));
                        // Serial.println("Linear [m/s]: L " + String(vel->angular2linear(leftSpeed, WHEEL_RADIUS_LEFT), 3) + 
                        //             " | R " + String(vel->angular2linear(rightSpeed, WHEEL_RADIUS_RIGHT), 3) );
                    }

                }
                break;
            }
            // case ('5'):
            // {
            //     Serial.println("========= Go by magnetic line. Debug =========");
            //     // resertPosition();
            //     // goMagneticLine(del, true);
            //     Serial.println("Coming soon");
            //     globalStop = true;
            //     break;
            // }
            // case ('6'):
            // {
            //     Serial.println("========= Go by magnetic line. Go to point =========");
            //     // resertPosition();
            //     // goMagneticLine2Point(3, -1, del, deb);
            //     // if (!globalStop) {
            //     //   stopMoving();
            //     //   globalStop = true;
            //     //   Serial.println("Fatal error");
            //     // }
            //     Serial.println("Coming soon");
            //     globalStop = true;
            //     break;
            // }
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
    float speed = vel->getMaxWheel();      //vel->linear2angular(MAX_LIN_SPEED, wheelRadius);
    Serial.println("set speed " + String(speed));
    float deltaAngL;
    float deltaAngR;

    //float distWheelL = 0.0;
    //float distWheelR = 0.0;
    //float distWheelC = 0.0;

    while (!globalStop)
    {
        timersTick();

        inByte = getSerialData();
        globalSerialControl(inByte);
        switch (inByte)
        {
        case ('w'):
            driveMoving(speed, speed);
            break;
        case ('s'):
            driveMoving(-speed, -speed);
            break;
        case (' '):
            stopMoving();
            break;
        case ('d'):
            driveMoving(speed, -speed);
            break;
        case ('a'):
            driveMoving(-speed, speed);
            break;
        case ('e'):
            if (speed <= vel->getMaxWheel() - 5)
                speed += 5;
            break;
        case ('q'):
            if (speed >= 5)
                speed -= 5;
            break;
        }

        deltaAngL = motorBlockL->getAngleOffset();
        deltaAngR = motorBlockR->getAngleOffset();

        //distWheelL = distWheelL + deltaAngL * wheelRadius;
        //distWheelR = distWheelR + deltaAngR * wheelRadius;
        //distWheelC = (distWheelR + distWheelL) / 2;

        pos->estCurrentPosition(deltaAngL, deltaAngR, wheelRadius, baseLength);

        if (msgTimer->ready()) {
            Serial.println(String(pos->x, 3) + " " + String(pos->y, 3) + " " + String(pos->alpha));
        }
    }
}

void TwoWheeledRobot::resertPosition()
{
    pos->x = 0.0;
    pos->y = 0.0;
    pos->alpha = 0.0;
    resertMotorsPID();
}

void TwoWheeledRobot::resertMotorsPID()
{
    motorBlockL->resetErrPID();
    motorBlockR->resetErrPID();
}

void TwoWheeledRobot::setPositionAndStartMove(float _x, float _y, int del, bool deb)
{
    // задаём конечную позицию в контроллере
    motionControler->setTargetPosition(_x, _y);
    Serial.println("Target pos: [" + String(_x, 3) + ", " + String(_y, 3) + "]");
    Serial.println("Start move to target position");
    moveToTargetPosition(del, deb);
}

void TwoWheeledRobot::setPositionAndStartMove(float _x, float _y, float beta, int del, bool deb)
{
    // задаём конечную позицию в контроллере
    motionControler->setTargetPosition(_x, _y, beta);
    Serial.println("Target pos: [" + String(_x, 3) + ", " + String(_y, 3) + "], finish heading angle: " + beta);
    Serial.println("Start move to target position");
    moveToTargetPosition(del, deb);
}

void TwoWheeledRobot::moveToTargetPosition(int del, bool deb)
{
    // задаём текущую позицию в контроллере
    motionControler->setActualPosition(pos->x, pos->y, pos->alpha);

    // поворот колёс за время между оценкой положения робота
    float deltaAngL = 0.0;
    float deltaAngR = 0.0;

    while (!motionControler->isReachPosition())
    {
        timersTick();

        // прерывание на консоль
        globalSerialControl();
        if (globalStop)
            break;

        if (discretTimer->ready())  
        { 
            // вычисляем новую скорость для достижения точки
            motionControler->updateVelocity();

            // Временное решение по ограничению ускорения движения на участках
            float realLinSpeed = (motorBlockL->getLastSpeed()*WHEEL_RADIUS_LEFT + motorBlockR->getLastSpeed()*WHEEL_RADIUS_RIGHT) / 2;
            float desiredLinSpeed = motionControler->get_v();
            float accelerationLin = abs((desiredLinSpeed - realLinSpeed) / ((float)discretTimer->getPrd() / 1000.0f));  // [m/s^2]
            float realAngSpeed = 2 * (motorBlockR->getLastSpeed() * WHEEL_RADIUS_RIGHT - realLinSpeed) / BASE_LENGTH;
            float desiredAngSpeed = motionControler->get_w();
            float accelerationAng = abs((desiredAngSpeed - realAngSpeed) / ((float)discretTimer->getPrd() / 1000.0f));  // [rad/s^2]
            if (accelerationLin > MAX_LIN_ACCEL)
            {
                float correlation = MAX_LIN_ACCEL / accelerationLin;
                vel->wheelL = motionControler->get_wheelL() * correlation;
                vel->wheelR = motionControler->get_wheelR() * correlation;
                //Serial.println("Lin " + String(correlation));
            }
            else if (accelerationAng > MAX_ANG_ACCEL)
            {
                float correlation = MAX_ANG_ACCEL / accelerationAng;
                vel->wheelL = motionControler->get_wheelL() * correlation;
                vel->wheelR = motionControler->get_wheelR() * correlation;
                //Serial.println("Ang " + String(correlation));
            }
            else
            {
                vel->wheelL = motionControler->get_wheelL();
                vel->wheelR = motionControler->get_wheelR();
            }

            if (!deb)
                driveMoving(vel->wheelL, vel->wheelR);
            
            // обновляем текущую позицию по колёсной одометрии
            deltaAngL = motorBlockL->getAngleOffset();
            deltaAngR = motorBlockR->getAngleOffset();
            pos->estCurrentPosition(deltaAngL, deltaAngR, wheelRadius, baseLength);
            // обновляем текущую позицию в контроллере
            motionControler->setActualPosition(pos->x, pos->y, pos->alpha);
        }

        if (msgTimer->ready()) 
        {
            //Serial.print("K " + String(motionControler->get_K(), 3));
            Serial.print("Delta " + String(motionControler->get_delta(), 3) + ". Distance " + String(motionControler->get_distance(), 3));
            Serial.print("Linear " + String(motionControler->get_v(), 3) +". Angular " + String(motionControler->get_w(), 3));
            Serial.println(". Wheel speed: " + String(motionControler->get_wheelL(), 2) + " | " + String(motionControler->get_wheelR(), 2) +
                            ". Actual: [" + String(pos->x, 3) + ", " + String(pos->y, 3) + "], heading: " + String(pos->alpha, 3));
        }
    }

    if (motionControler->isReachPosition()) {
        Serial.println("TARGET POINT REACHED");
        Serial.println("X: " + String(pos->x) + " Y: " + String(pos->y));
    } else {
        Serial.println("Early exit!");
    }
    stopMoving();
}

void TwoWheeledRobot::goCWtest(float L, int del, bool deb)
{
    resertPosition();
    setPositionAndStartMove(L, 0.0, del, deb);
    setPositionAndStartMove(L, -L, del, deb);
    setPositionAndStartMove(0.0, -L, del, deb);
    setPositionAndStartMove(0.0, 0.0, del, deb);
    Serial.println("CW track finish");
}

void TwoWheeledRobot::goCCWtest(float L, int del, bool deb)
{
    resertPosition();
    setPositionAndStartMove(L, 0.0, del, deb);
    setPositionAndStartMove(L, L, del, deb);
    setPositionAndStartMove(0.0, L, del, deb);
    setPositionAndStartMove(0.0, 0.0, del, deb);
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

//     deltaAngL = motorBlockL->getAngleOffset();
//     deltaAngR = motorBlockR->getAngleOffset();

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

//     deltaAngL = motorBlockL->getAngleOffset();
//     deltaAngR = motorBlockR->getAngleOffset();

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

void TwoWheeledRobot::driveMoving(float velL, float velR)
{
    motorBlockL->updateVelocity(velL);
    motorBlockR->updateVelocity(velR);
}

void TwoWheeledRobot::timersStart() {
    discretTimer->start();
    motorTimer->start();
    msgTimer->start();  
    
    motorBlockL->tickPrepare();
    motorBlockR->tickPrepare();  
}

void TwoWheeledRobot::timersTick() {
    discretTimer->tick();
    motorTimer->tick();
    msgTimer->tick();
    motorBlockL->tick();
    motorBlockR->tick();

    if (motorTimer->ready())
    {
        motorBlockL->updatePWM();
        motorBlockR->updatePWM();
    }
}