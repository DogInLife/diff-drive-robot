#include "twoWheeledRobot.h"
#include "constants.h"

#define RST_PIN         8          // Configurable, see typical pin layout above
#define SS_PIN          53         // Configurable, see typical pin layout above

TwoWheeledRobot::TwoWheeledRobot()
  :reachedGoal(false), globalStop(true),
  PIN_CURRENT_SENSOR(A12),
  inByte(0), newMinRange(0)
{
  // RFID READER
  if(!Serial) 
    Serial.begin(38400);

  rfidReader = new RFIDReader(SS_PIN, RST_PIN);
  rfidReader->readerStart();

  magneticLineReader = new MagneticLineReader();

  motorBlockL = new MotorBlock();
  motorBlockR = new MotorBlock();
  pidL = new PID();
  pidR = new PID();
  pid = new PID();
  pinMode(PIN_CURRENT_SENSOR, LOW);
  
  motionControler = new MotionControler();
}

TwoWheeledRobot::~TwoWheeledRobot()
{
  delete rfidReader;

  delete motorBlockL;
  delete motorBlockR;
  delete pidL;
  delete pidR;
  delete pid;
}

void TwoWheeledRobot::createWheels(float wheelRadius, float baseLength, float maxVel)
{
  motorBlockL->createWheel(wheelRadius);
  motorBlockR->createWheel(wheelRadius);
  this->baseLength = baseLength;
  // vel.max = 6.28/60*wheelRadius*maxVel;
  vel.maxWheel = maxVel; // макс. скорость вращения колёс [об/мин]
  //vel.maxRobot = maxVel * 2.0 * 3.141593 * wheelRadius / 60.0; 
  vel.maxRobot = 2 * PI * wheelRadius * maxVel / 60; // макс. линейная скорость робота [м/с]

  Serial.print("max wheel speed: " + String(vel.maxWheel) + "max robot speed: " + String(vel.maxRobot));
  // if (DEBUG){
  //   
  // }
  motionControler->setSpeed(vel.maxRobot);
}

// === SET ===
void TwoWheeledRobot::setEncoderPins(byte encPinL, byte encPinR)
{
  motorBlockL->setEncorerPin(encPinL);
  motorBlockR->setEncorerPin(encPinR);
}

void TwoWheeledRobot::setDriverPins(byte driverPinPWM_R, byte driverPin_R2, byte driverPin_R1, byte driverPin_L1, byte driverPin_L2, byte driverPinPWM_L)
{
  motorBlockL->setDriverPin(driverPin_L1, driverPin_L2, driverPinPWM_L);
  motorBlockR->setDriverPin(driverPin_R1, driverPin_R2, driverPinPWM_R);
}

void TwoWheeledRobot::tuneWhlPID(float KpL, float KiL, float KdL, float KpR, float KiR, float KdR)
{
   pidL->setCoefficient(KpL, KiL, KdL);
   pidR->setCoefficient(KpR, KiR, KdR);
}

void TwoWheeledRobot::tunePID(float Kp, float Ki, float Kd)
{
  pid->setCoefficient(Kp, Ki, Kd);
}

// === GET ===
float TwoWheeledRobot::getRadiusWheels()
{
  return motorBlockL->getRadiusWheels();
}

byte TwoWheeledRobot::getSerialData()
{
  return Serial.read();
}

void TwoWheeledRobot::serialControl(int del, bool deb) {
  while (true)
  {
    if (globalStop)
    {
      stopMoving();
      Serial.println(" ===== Choose mode ===== ");
      Serial.println(" 1 = Manual control ");
      Serial.println(" 2 = Turn to left 90 deg");
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
        Serial.println("=== You are using manual control ===");
        manualControl(10);
        break;
      
      case ('2'):
        Serial.println("========= Turn to left 90 deg =========");
        turnAngle(pos.theta + PI/2, del, deb);
        break;

      case ('3'):
        Serial.println("====== CW trajectory ======");
        goCWtest(1.0, del, deb);
        break;
      
      case ('4'):
        Serial.println("====== CCW trajectory ======");
        goCCWtest(0.8, del, deb);
        break;

      case ('5'):
        Serial.println("========= Go to NULL position =========");
        //goToNULL(del, deb);
        break;

      case ('6'):
        Serial.println("========= Go by magnetic line. Debug =========");
        resertPosition();
        goMagneticLine(del, true);
        break;

      case ('7'):
        Serial.println("========= Go by magnetic line. Go to point =========");
        resertPosition();
        goMagneticLine2Point(3, -1, del, deb);
        if (!globalStop) {
          stopMoving();
          globalStop = true;
          Serial.println("Fatal error");
        }
        break;

      // case ('5'):
      //   Serial.println("========= GO GO GO =========");
      //   goToGoal(1.0, 0.0, true, del, deb, false, 3);
      //   break;
      
      // case ('6'):
      //   Serial.println("====== Circle trajectory ======");
      //   goCircle(0.75, del, deb, 2);
      //   break;

      // case ('7'):
      //   Serial.println(" ===== Rotation test ===== ");
      //   rot_test(60, del, deb, 0.61, 0.61);
      //   break;

      // case ('8'):
      //   Serial.println(" ===== RFID reader test ===== ");
      //   rfidTest(del);
      //   break;
    }
  }
}

void TwoWheeledRobot::globalSerialControl()
{
  globalSerialControl(getSerialData());
}

void TwoWheeledRobot::globalSerialControl(byte inB)
{
  switch(inB)
  {
    case('z'):
      stopMoving();
      globalStop = true;
    break;
  }
}

// ====================== robot behavior ===================== //
// del - частота обновления
void TwoWheeledRobot::goToPosition(float x, float y, int del, bool deb) {
  
  bool isFinish = false;

  // поворот колёс за время между оценкой положения робота
  float deltaAngL = 0.0;
  float deltaAngR = 0.0;
  // расстояние пройденное колесом
  float distWheelL = 0.0;
  float distWheelR = 0.0;
  float distWheelC = 0.0;             // общее
  // скорректированные скорости колёс
  float velL;
  float velR;
  // допуск по достижению конченой точки
  float posThreshold = 0.025;

  float r = getRadiusWheels();
  float l = baseLength;
  float err = 0.0;
  // измерительный промежуток 
  //float dt = del / 1000.0;
  float dt = del;

  while(!isFinish && !globalStop) {
    globalSerialControl();
    
    // расчет угла, на котором расположена целевая точка
    pos.thetaGoal = atan2(y-pos.y, x-pos.x);

    err = pid->computeAngleError(pos.thetaGoal, pos.theta);
    
    vel.ang = pid->computeControl(err, dt);
    vel.lin = vel.computeLinearSpeed(err);
    Serial.println("err " + String(err) + "ang " + String(vel.ang) + " | lin " + String(vel.lin));

    velL = (2.0 * vel.lin - vel.ang * l) / (2.0 * r);
    velR = (2.0 * vel.lin + vel.ang * l) / (2.0 * r);
    
    //Serial.println("velL: " + String(velL, 3) + " velR " + String(velR, 3));

    if(!deb)
      goForward(velL, velR);

    deltaAngL = motorBlockL->getDeltaAngle();
    deltaAngR = motorBlockR->getDeltaAngle();

    distWheelL = distWheelL + deltaAngL*r;
    distWheelR = distWheelR + deltaAngR*r;
    distWheelC = (distWheelR + distWheelL) / 2;

    pos.estCurrentPosition(deltaAngL, deltaAngR, r, l);
    //String msg_pos = "X: " + String(pos.x, 3) + " Y: " + String(pos.y, 3) + " Th: " + String(pos.theta, 3);
    //Serial.println(msg_pos);

    if((abs(x-pos.x) < posThreshold) && (abs(y-pos.y) < posThreshold)) {
      stopMoving();
      Serial.println("TARGET POINT REACHED");
      Serial.println("X: " + String(pos.x) + " Y: " + String(pos.y));
      break;
    }
  
    delay(del);
  }
}

void TwoWheeledRobot::turnAngle(float theta, int del, bool deb) {
  
  // поворот колёс за время между оценкой положения робота
  float deltaAngL = 0.0;
  float deltaAngR = 0.0;
  // скорректированные скорости колёс
  float velL;
  float velR;
  // допуск по достижению конченого угла
  float anglThreshold = 0.025;

  float r = getRadiusWheels();
  float l = baseLength;
  float err = 0.0;

  // измерительный промежуток 
  //float dt = del / 1000.0;
  float dt = del;

  while(!globalStop) {
    globalSerialControl();
    
    // расчет угла, на котором расположена целевая точка
    pos.thetaGoal = theta;

    err = pid->computeAngleError(pos.thetaGoal, pos.theta);
    
    vel.ang = pid->computeControl(err, dt);
    vel.lin = vel.computeLinearSpeed(err);

    velL = (2.0 * vel.lin - vel.ang * l) / (2.0 * r);
    velR = (2.0 * vel.lin + vel.ang * l) / (2.0 * r);
     
    if(!deb)
      goForward(velL, velR);

    deltaAngL = motorBlockL->getDeltaAngle();
    deltaAngR = motorBlockR->getDeltaAngle();

    pos.estCurrentPosition(deltaAngL, deltaAngR, r, l);
    Serial.println(" Th: " + String(pos.theta));

    if((abs(err) < anglThreshold)) {
      stopMoving();
      Serial.println("ANGLE REACHED");
      Serial.println("X: " + String(pos.x) + " Y: " + String(pos.y) + + " Th: " + String(pos.theta));
      break;
    }
  
    delay(del);
  }
}

void TwoWheeledRobot::resertPosition() {
  pos.x = 0.0;
  pos.y = 0.0;
  pos.theta = 0.0;
  pid->resetErr();
}

void TwoWheeledRobot::goToNULL(int del, bool deb) {
  float x, y, theta;
  int get_numbers = 0;
  while(get_numbers < 3)
  {
    //getSerialData()
    if (Serial.available()) {
      int data = Serial.read();
      if (data) {
        Serial.println(data);
        switch (get_numbers)
        {
          case 0:
            x = data;
            break;
          case 1:
            y = data;
            break;
          case 2:
            theta = data;
            break;
        }
        get_numbers++;
      }
    }
  }
  Serial.print("finish");
  Serial.println("x " + String(x) + " | y " + String(y) + " | theta " + String(theta));
  // goToPosition(x, y, del, deb);
  // turnAngle(theta, del, deb);
  resertPosition();
}

/*
void TwoWheeledRobot::goTrack(Position points[], int del, bool deb) {
  size_t size  = sizeof points / sizeof(Position);
  for (int i = 0; i < size; i++)
  {
    if (!globalStop)
      goToPosition(points[i].x, points[i].y);
  }
}
*/
// ======= CW ======== //
// L - сторона квадратной траектории
void TwoWheeledRobot::goCWtest(float L, int del, bool deb) {
  resertPosition();
  goToPosition(L,   0.0, del, false);
  goToPosition(L,   -L,  del, false);
  goToPosition(0.0, -L,  del, false);
  goToPosition(0.0, 0.0, del, false);
  turnAngle(0.0, del, false);
  Serial.println("CW track finish");
  Serial.println("X: " + String(pos.x) + " Y: " + String(pos.y) + " Th: " + String(pos.theta));
}

// ======= CCW ======== //
// L - сторона квадратной траектории
void TwoWheeledRobot::goCCWtest(float L, int del, bool deb) {  
  resertPosition();
  goToPosition(L,   0.0, del, false);
  goToPosition(L,   L,   del, false);
  goToPosition(0.0, L,   del, false);
  goToPosition(0.0, 0.0, del, false);
  turnAngle(0.0, del, false);
  Serial.println("CCW track finish");
  Serial.println("X: " + String(pos.x) + " Y: " + String(pos.y) + " Th: " + String(pos.theta));
}

void TwoWheeledRobot::goMagneticLine(int del, bool deb) {
    // поворот колёс за время между оценкой положения робота
  float deltaAngL = 0.0;
  float deltaAngR = 0.0;
  // скорректированные скорости колёс
  float velL;
  float velR;

  float r = getRadiusWheels();
  float l = baseLength;
  float err = 0.0;

  // измерительный промежуток 
  //float dt = del / 1000.0;
  float dt = del;

  // Направление. Угол, расстояние до точки
  float delta, distance;

  magneticLineReader->updateAverageSignal();
  size_t count = magneticLineReader->getSensorCount();
  size_t middle_sen_i = count / 2 - 1;

  String message = "";

  while(!globalStop) {
    globalSerialControl();

    byte* checkResult = magneticLineReader->checkMagneticField();
    
    byte leftDetect = 0;
    for (size_t i = 0; i <= middle_sen_i; i++) {
      leftDetect += checkResult[i];
    }
    byte rightDetect = 0;
    for (size_t i = middle_sen_i+1; i < count; i++) {
      rightDetect += checkResult[i];
    }
  
    // Движение прямо
    delta = 0;
    distance = magneticLineReader->hallSensors[middle_sen_i]->x;
    if (leftDetect && rightDetect &&
        ((leftDetect > rightDetect && leftDetect/rightDetect < 1.5) ||
         (leftDetect < rightDetect && rightDetect/leftDetect < 1.5))) {
      if (leftDetect+rightDetect < 4 && checkResult[middle_sen_i] && checkResult[middle_sen_i+1]) {
        message = "Forward";
      } else {
        message = "Crossroads";
        //stopMoving();
        //globalStop = true;
      }
    }
    else if (leftDetect && leftDetect > rightDetect) {
      if (leftDetect == 1 && checkResult[middle_sen_i]) {
        message = "Forward";
      } else {
        message = "Left to ";
        for (size_t i = 0; i <= middle_sen_i; i++) {
          if (checkResult[i]) {
            message += String(i);
            delta = atan(magneticLineReader->hallSensors[i]->y/magneticLineReader->hallSensors[i]->x);
            distance = sqrt(pow(magneticLineReader->hallSensors[i]->x, 2) + pow(magneticLineReader->hallSensors[i]->y, 2));
            break;
          }
        }
      }
    }
    else if (rightDetect && rightDetect > leftDetect) {
      if (rightDetect == 1 && checkResult[middle_sen_i+1]) {
        message = "Forward";
      } else {
        message = "Right to ";
        for (size_t i = count-1; i > middle_sen_i; i--) {
          if (checkResult[i]) {
            message += String(i);
            delta = atan(magneticLineReader->hallSensors[i]->y/magneticLineReader->hallSensors[i]->x);
            distance = sqrt(pow(magneticLineReader->hallSensors[i]->x, 2) + pow(magneticLineReader->hallSensors[i]->y, 2));
            break;
          }
        }
      }      
    }
    else {
      message = "NoLine";
    }
    
    motionControler->calculate(delta, distance, 0);
    
    velL = (motionControler->get_v() - l*motionControler->get_w()/2)/r;
    velR = (motionControler->get_v() + l*motionControler->get_w()/2)/r;
    //Serial.println("get_v: " + String(motionControler->get_v()) + ". get_w: " + String(l*motionControler->get_w()/2));
    
    if (deb)
      Serial.println(message);
    else
      goForward(velL*20, velR*20);
      //goForward(velL, velR);

    deltaAngL = motorBlockL->getDeltaAngle();
    deltaAngR = motorBlockR->getDeltaAngle();

    pos.estCurrentPosition(deltaAngL, deltaAngR, r, l);

    delay(del);
  }
  magneticLineReader->takeOffLed();
}

void TwoWheeledRobot::goMagneticLine2Point(float _x, float _y, int del, bool deb) {
  // поворот колёс за время между оценкой положения робота
  float deltaAngL = 0.0;
  float deltaAngR = 0.0;
  // скорректированные скорости колёс
  float velL;
  float velR;

  float r = getRadiusWheels();
  float l = baseLength;
  float err = 0.0;

  // измерительный промежуток 
  float dt = del;

  // Направление. delta - угол, distance - расстояние до точки
  float delta, distance;

  size_t count = magneticLineReader->getSensorCount();
  size_t middle_sen_i = count / 2 - 1;

  String message = "";

  int crossroadsCounter = 0;                          // Счётчик перекрёстков
  float actualDistance = 0, coverDistance = 0, admissionDistance = 0.005;  // Дистанции для пересечения перекрёстка, mm
  float actualAngle = 0, coverAngle = 0, admissionAngle = 0.06;  // Дистанции для пересечения перекрёстка, mm
  bool isManeuvering = false;                         // Робот в процессе выполнения манёвра
  int decision;                                       // Указание к действию

  while(!globalStop) {
    globalSerialControl();
    
    if (isManeuvering) {  // Если выполняется манёвр
      if (crossroadsCounter <= _x)
        actualDistance = pos.x;
      else 
        actualDistance = -pos.y;
      if (coverDistance - actualDistance < admissionDistance) { // Если доехали до центра перекрёстка
        if (decision == 1) {                          // Если манёвр "Движение вперёд"
          isManeuvering = false;    // Движение вперёд окончено
          message = "Finish crossroad: " + String(crossroadsCounter);
        } else if (decision == 2 || decision == 3) {  // Если манёвр "Поворот"
          actualAngle = pos.theta;
          if (abs(coverAngle - actualAngle) < admissionAngle) { // Если повернули
            isManeuvering = false;  // Поворот окончен
            message = "Finish crossroad: " + String(crossroadsCounter);
          }
          else {                                                // Иначе продолжаем поворачиваться
            delta = coverAngle - actualAngle;
            distance = 0;
            message = "actualAngle " + String(actualAngle) + " | coverAngle " + String(coverAngle)  + " | delta " + String(delta);
          }
        } else {                                      // Иначе остановка        
          isManeuvering = false;         
          stopMoving();
          globalStop = true;
          message = "Finish. Last crossroad: " + String(crossroadsCounter);
        }
      }
      else {                                                    // Иначе продолжаем движение прямо
        delta = 0;
        distance = coverDistance - actualDistance;
        message = "actualDistance " + String(actualDistance) + " | coverDistance " + String(coverDistance);
      }
    }
    else {                // Иначе обычное следование линии     
      // Опрос датчика магнитной линии 
      byte* checkResult = magneticLineReader->checkMagneticField(); 
      byte leftDetect = 0;
      for (size_t i = 0; i <= middle_sen_i; i++) {
        leftDetect += checkResult[i];
      }
      byte rightDetect = 0;
      for (size_t i = middle_sen_i+1; i < count; i++) {
        rightDetect += checkResult[i];
      }
      
      /*  Если засекли линию слева и права
          И линия не сильно смещена от центра,
          то движемся по середине линии или засекли перекрёсток */
      if (leftDetect && rightDetect &&      
          ((leftDetect > rightDetect && leftDetect/rightDetect < 2) ||
          (leftDetect < rightDetect && rightDetect/leftDetect < 2))) {    
        if (leftDetect+rightDetect > 3) {   // Если площадь детектирования большая, то засекли перекрёсток 
          message = "Crossroad";
          decision = moveDecision(_x, _y, ++crossroadsCounter);
          // Дистанция до пересечения центра перекрёстка и центра робота
          //actualDistance = sqrt(pos.x*pos.x + pos.y*pos.y);
          if (crossroadsCounter <= _x)
            actualDistance = pos.x;
          else 
            actualDistance = -pos.y;
          coverDistance = actualDistance + magneticLineReader->hallSensors[middle_sen_i]->x;
          // Угол поворота на перекрёстке
          if (decision == 0)
            message += ". Finish";
          else if (decision == 1)
            message += ". Forward";
          else if (decision == 2) {
            actualAngle = pos.theta;
            coverAngle = actualAngle + PI/2;
            message += ". Left";
          } else if (decision == 3) {
            actualAngle = pos.theta;
            coverAngle = actualAngle - PI/2;
            message += ". Right";
          }
          // Начало манёвра
          isManeuvering = true;
          Serial.println(message);
          continue;
        }        
        else {                              // Продолжаем движение по центру линии
          message = "Forward";
          delta = 0.;
          distance = magneticLineReader->hallSensors[middle_sen_i]->x;
        }
      } else if (leftDetect && leftDetect > rightDetect) {      // Если преобладает левое направление
        if (leftDetect == 1 && checkResult[middle_sen_i]) {     // Если линия находится по центру, движемся прямо
          message = "Forward";
          delta = 0.;
          distance = magneticLineReader->hallSensors[middle_sen_i]->x;
        } else {                                                // Иначе поворот налево
          message = "Left";
          for (size_t i = 0; i <= middle_sen_i; i++) {
            if (checkResult[i]) {
              delta = atan(magneticLineReader->hallSensors[i]->y/magneticLineReader->hallSensors[i]->x);
              distance = sqrt(pow(magneticLineReader->hallSensors[i]->x, 2) + pow(magneticLineReader->hallSensors[i]->y, 2));
              break;
            }
          }
        }
      }
      else if (rightDetect && rightDetect > leftDetect) {       // Если преобладает правое направление
        if (rightDetect == 1 && checkResult[middle_sen_i+1]) {  // Если линия находится по центру, движемся прямо
          message = "Forward";
          delta = 0.;
          distance = magneticLineReader->hallSensors[middle_sen_i]->x;
        } else {                                                // Иначе поворот направо
          message = "Right";
          for (size_t i = count-1; i > middle_sen_i; i--) {     
            if (checkResult[i]) {
              delta = atan(magneticLineReader->hallSensors[i]->y/magneticLineReader->hallSensors[i]->x);
              distance = sqrt(pow(magneticLineReader->hallSensors[i]->x, 2) + pow(magneticLineReader->hallSensors[i]->y, 2));
              break;
            }
          }
        }
      }
      else {
        message = "NoLine";
        delta = 0.;
        distance = magneticLineReader->hallSensors[middle_sen_i]->x;
      }
    }

    motionControler->calculate(delta, distance, 0);
    
    if (isManeuvering && distance == 0 && (decision == 2 || decision == 3)) {
      velL = (motionControler->get_v() + l*motionControler->get_w()/2/r) * (decision == 2 ? -2 : 2);
      velR = -velL;
      Serial.println(String(motionControler->get_v()) + " " + String(l*motionControler->get_w()/2/r));
    }
    else {
      velL = (motionControler->get_v() - l*motionControler->get_w()/2)/r;
      velR = (motionControler->get_v() + l*motionControler->get_w()/2)/r;
    }
    //Serial.println("get_v: " + String(motionControler->get_v()) + ". get_w: " + String(l*motionControler->get_w()/2));
    
    velL = linear2angular(velL);
    velR = linear2angular(velR);
    
    if (message.compareTo("NoLine")) {
      //Serial.println(message);
      Serial.println("velL: " + String(velL) + " | velR: " + String(velR) + (" (rpm)") );
    }

    if (!deb)
      goForward(velL, velR);

    deltaAngL = motorBlockL->getDeltaAngle();
    deltaAngR = motorBlockR->getDeltaAngle();

    pos.estCurrentPosition(deltaAngL, deltaAngR, r, l);

    delay(del);
  }
  magneticLineReader->takeOffLed();
}

int TwoWheeledRobot::moveDecision(float _x, float _y, int count) {
  if (count == _x) {
    if (_y > 0)
      return 2;
    else if (_y < 0)
      return 3;
    else
      return 0;
  }
  else if (count > _x) {
    if (count < _y)
      return 1;
    else
      return 0;
  }
  else {
    return 1;
  }
}

void TwoWheeledRobot::rfidTest(int del) {
  int rfidFound;

  while(true) {
    globalSerialControl();
    rfidFound = rfidReader->checkReaderData();
    if(rfidFound > 0)
      Serial.println(rfidFound);

    delay(del);
  }
}

void TwoWheeledRobot::goCircle(float radius, int ptsNum, bool deb, int circles)
{
  float x0 = 0.0;
  float y0 = 0.0;

  float x;
  float y;
  
  bool followRFID = false;
  //bool followRFID = true;

  bool isFinish = false;

  float dPhi = 2.0*3.141593 / ptsNum;
  
  int rfidFound;

  //long t_start = millis();
  for(int c = 1; c <= circles; c++) {
    Serial.println(" CIRCLE " + String(c));
    for(int i=1; i <= ptsNum; i++)
    {
      globalSerialControl();
      //if(i % 2 == 0) followRFID = true;
      if(c == circles && i == ptsNum) { isFinish = true; }
      x = x0 + radius * sin(dPhi*i);
      y = (y0 + radius) - radius * cos(dPhi*i);
      Serial.println("X" + String(i) + ": " + String(x, 3) + " Y" + String(i) + ": " + String(y, 3));
      rfidFound = goToGoal(x, y, isFinish, 50, deb, followRFID, i);
      if(globalStop) { 
        Serial.println(" ==== GLOBAL STOP ==== ");
        break; 
      }

      if(rfidFound > 0) {
        if(rfidFound <= 2)
          i = ptsNum/2;
        else if(i > ptsNum/2)
          i = ptsNum;
      }
    }
    //isFinish = false;
  }
}

// void TwoWheeledRobot::faceToGoal(float xGoal, float yGoal) {
//   bool isFaced = false;
//   float err = 0.0;
  
//   //Расчет угла, на котором расположена целевая точка
//   pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);
//   err = pid->e
//   while()
// }

// void TwoWheeledRobot::followRFIDs(int numRFIDs, bool deb) {
//   for(int i=1; i<=numRFIDs; i++) {
//     goToGoal():
//     if(globalStop) {
//       Serial.println(" ==== GLOBAL STOP ==== ");
//       break
//     }
//   }
// }

// ====================== robot behavior ===================== //
// ======= GO ======== //
int TwoWheeledRobot::goToGoal(float xGoal, float yGoal, bool isFinish, int del, bool deb, bool followRFID, int idRFID) {


  reachedGoal = false;

  // поворот колёс за время между оценкой положения робота
  float deltaAngL = 0.0;
  float deltaAngR = 0.0;

  // float velL_meas = 0.0;
  // float velR_meas = 0.0;

  // float V_err = 0.0;

  float distWheelL = 0.0;
  float distWheelR = 0.0;
  float distWheelC = 0.0;

  // скорректированные скорости колёс
  float velL;
  float velR;

  float posThreshold = 0.025;

  //Расчет угла, на котором расположена целевая точка
  pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);

  float r = getRadiusWheels();
  float L = baseLength;
  float err = 0.0;

  float dt = 0.0;

  int rfidFound = 0;

  while(!reachedGoal && !globalStop) {
    //pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);
    //Serial.println("Theta goal: " + String(pos.thetaGoal, 3) + " Theta: " + String(pos.theta, 3));

    //t_curr = millis() - t_start;
    //dt = (t_curr - t_prev) / 1000.0;

    dt = (del) / 1000.0; 
    //Serial.println(dt);
    err = pid->computeAngleError(pos.thetaGoal, pos.theta);
    //Serial.println("Err theta: " + String(err, 3));

    vel.ang = pid->computeControl(err, dt);
    //vel.ang = pid->computeControl(err, dt, rfidFound);
    vel.lin = vel.computeLinearSpeed(err);

    // velL_meas = (deltaAngL * 60000.0) / (2.0 * 3.141593 * del);
    // velR_meas = (deltaAngR * 60000.0) / (2.0 * 3.141593 * del);

    // Serial.println(String(velL_meas, 3) + "  " + velR_meas);
    

    // String msg_vel = "Ang_Vel: " + String(vel.ang, 3) + " Lin_Vel: " + String(vel.lin, 3);
    // Serial.println(msg_vel);

    //Расчет скоростей для каждого двигателя в об/мин
    // velL = ((2.0 * vel.lin - vel.ang * L) / (2.0 * r)) * 60.0 / (2*3.141593);
    // velR = ((2.0 * vel.lin + vel.ang * L) / (2.0 * r)) * 60.0 / (2*3.141593);
    velL = (2.0 * vel.lin - vel.ang * L) / (2.0 * r);
    velR = (2.0 * vel.lin + vel.ang * L) / (2.0 * r);
    
    //Serial.println("velL: " + String(velL, 3) + " velR " + String(velR, 3));

    // motorBlockL->setVelocity(velL, vel.maxWheel, newMinRange);
    // motorBlockR->setVelocity(velR, vel.maxWheel, newMinRange);

    if(!deb)
      goForward(velL, velR);

    deltaAngL = motorBlockL->getDeltaAngle();
    deltaAngR = motorBlockR->getDeltaAngle();

    distWheelL = distWheelL + deltaAngL*r;
    distWheelR = distWheelR + deltaAngR*r;
    distWheelC = (distWheelR + distWheelL) / 2;

    pos.estCurrentPosition(deltaAngL, deltaAngR, r, L);
    String msg_pos = "X: " + String(pos.x, 3) + " Y: " + String(pos.y, 3) + " Th: " + String(pos.theta, 3);
    Serial.println(msg_pos);

    // pos.correctPosEst(distWheelC);
    // if(pos.corrected) {
    //   Serial.println("CORRECTED X: " + String(pos.x, 3) + " Y: " + String(pos.y, 3));
    //   pos.corrected = false;
    // }

    if((abs(xGoal-pos.x) < posThreshold) && (abs(yGoal-pos.y) < posThreshold)) {
      //Serial.println("PT REACHED");
      //Serial.println("X_e: " + String(xGoal-pos.x, 3) + " Y_e: " + String(yGoal-pos.y, 3) + " Theta: " + String(pos.theta, 3));
      reachedGoal = true;
    }

    rfidFound = rfidReader->checkReaderData();
    switch (rfidFound) {
      case 0:
        break;

      //Serial.println("X_err: " + String(xGoal-pos.x, 3) + " Y_err: " + String(yGoal-pos.y));
      case 1:
        // pos.x = (pos.x + 0.0)/2.0;
        // pos.y = (pos.y + 1.166)/2.0;
        pos.x = 0.0;
        //pos.y = 1.166;
        pos.y = 1.181;
        //pos.theta = (pos.theta + 3.1416) / 2.0;
        Serial.println("RFID 1 REACHED");
        break;
      case 2:
        // pos.x = (pos.x + 0.0)/2.0;
        // pos.y = (pos.y + 1.22)/2.0;
        pos.x = 0.0;
        // pos.y = 1.22;
        pos.y = 1.259;
        //pos.theta = (pos.theta + 3.1416) / 2.0;
        Serial.println("RFID 2 REACHED");
        break;
      case 3:
        // pos.x = (pos.x + 0.0)/2.0;
        // pos.y = (pos.y + 1.274)/2.0;
        pos.x = 0.0;
        pos.y = 0.038;
        //pos.theta = (pos.theta + 3.1416) / 2.0;
        Serial.println("BASE RFID 3 REACHED");
        break;
      case 4:
        // pos.x = (pos.x + 0.0)/2.0;
        // pos.y = (pos.y + 0.0)/2.0;
        pos.x = 0.0;
        pos.y = -0.038;
        //pos.theta = (pos.theta + 0.0) / 2.0;
        Serial.println("BASE RFID 4 REACHED");
        //reachedGoal = true;
        break;
    }

    if(rfidFound > 0) {
      //Serial.println("PREV: " + String(pos.thetaGoal));
      //pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);
      //Serial.println("CURR: " + String(pos.thetaGoal));
      Serial.println(String(pos.theta));

      return rfidFound;
    }
    // if(!followRFID) {
    //   if((abs(xGoal-pos.x) < 0.03) && (abs(yGoal-pos.y) < 0.03))
    //   {
    //     Serial.println("PT REACHED");
    //     Serial.println("X_e: " + String(xGoal-pos.x, 3) + " Y_e: " + String(yGoal-pos.y, 3) + " Theta: " + String(pos.theta, 3));
    //     reachedGoal = true;
    //     Serial.println(distWheelC);
    //   }
    // } else {
    //   rfidFound = rfidReader->checkReaderData();
    //   switch (rfidFound) {
    //     case 0:
    //       break;

    //     Serial.println("X_err: " + String(xGoal-pos.x, 3) + " Y_err: " + String(yGoal-pos.y));
    //     case 1:
    //       //pos.x = 0.6;
    //       //pos.y = 0.6;
    //       Serial.println("RFID 1 REACHED: 0.6 0.6");
    //       break;
    //     case 2:
    //       pos.x = 0.0;
    //       pos.y = 0.6;
    //       Serial.println("RFID 2 REACHED 0.0 0.6");
    //       break;
    //     case 3:
    //       pos.x = 0.0;
    //       pos.y = 1.2;
    //       Serial.println("RFID 3 REACHED 0.0 1.2");
    //       break;

    //     case 4:
    //       pos.x = 0.0;
    //       pos.y = 0.0;
    //       Serial.println("BACK TO BASE 0.0 0.0");
    //       reachedGoal = true;
    //       break;

    //     default:
    //       Serial.println("Stranger");
    //       break;
    //   }

    //   if(rfidFound == idRFID) {
    //     reachedGoal = true;
    //     //break;
    //   }
    // }

    // //Расчет угла, на котором расположена целевая точка
    // pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);
    // Serial.println("Theta GOAL: " + String(pos.thetaGoal, 3));

    if(reachedGoal)
    {
      if(isFinish)
      {
        stopMoving();
        Serial.println("TARGET POINT REACHED");
        //Serial.println("X_err: " + String(xGoal-pos.x, 3) + " Y_err: " + String(yGoal-pos.y));
        break;
      } 
      else { break; }
    }
  
    globalSerialControl();

    delay(del);
  }
  return 0;
}

// ############## Прямолинейное движение в пид-контроллером углов поворота колёс ###############
void TwoWheeledRobot::rot_test(int whl_vel_des, byte del, bool deb, float xGoal, float yGoal) {

  float r = getRadiusWheels();
  float R = 0.61; // радиус окружности
  float L = baseLength;

  float deltaAngL = 0.0;
  float deltaAngR = 0.0;

  float distWheelL = 0.0;
  float distWheelR = 0.0;
  float distWheelC = 0.0;

  float velL;
  float velR;

  vel.lin = whl_vel_des*r;
  vel.ang = vel.lin/R;

  //Serial.println("Desired // VelLin: " + String(vel.lin, 3) + " VelAng: " + String(vel.ang, 3));

  velL = (2.0 * vel.lin - vel.ang * L) / (2.0 * r);
  velR = (2.0 * vel.lin + vel.ang * L) / (2.0 * r);

  if(!deb)
    goForward(velL, velR);

  while(!reachedGoal && !globalStop) {
    deltaAngL = motorBlockL->getDeltaAngle();
    deltaAngR = motorBlockR->getDeltaAngle();

    distWheelL = distWheelL + deltaAngL*r;
    distWheelR = distWheelR + deltaAngR*r;
    distWheelC = (distWheelR + distWheelL) / 2;

    pos.estCurrentPosition(deltaAngL, deltaAngR, r, L);
    String msg_pos = "X: " + String(pos.x, 3) + " Y: " + String(pos.y, 3) + " Th: " + String(pos.theta, 3);
    Serial.println(msg_pos);

    //pos.correctPosEst(distWheelC);

    if(pos.corrected) {
      Serial.println("CORRECTED X: " + String(pos.x, 3) + " Y: " + String(pos.y, 3));
      pos.corrected = false;
    }

    if((abs(xGoal-pos.x) < 0.05) && (abs(yGoal-pos.y) < 0.05)) {
      Serial.println("PT REACHED");
      Serial.println("X_e: " + String(xGoal-pos.x, 3) + " Y_e: " + String(yGoal-pos.y, 3) + " Theta: " + String(pos.theta, 3));
      reachedGoal = true;
      Serial.println(distWheelC);
    }

      if(reachedGoal) {
        stopMoving();
        Serial.println("TARGET POINT REACHED");
        //Serial.println("X_err: " + String(xGoal-pos.x, 3) + " Y_err: " + String(yGoal-pos.y));
        break;
      }

      globalSerialControl();

      delay(del);
    }
  }


  // Serial.println("radius: " + String(radius, 3) + " R: " + String(R, 2) + " L: " + String(L, 3));

  // bool isReady = false;
  // bool isMoving = false;

  // float qL_curr = 0.0; // текущий измеренный угол поворота левого колеса
  // float qR_curr = 0.0; // текущий измеренный угол повората правого колеса
  // int t_curr = 0; // текущий момент времени

  // float qL_prev = 0.0; // предыдущий измеренный угол поворота левого колеса
  // float qR_prev = 0.0; // предыдущий измеренный угол поворота правого колеса
  // int t_prev = 0; // предыдущий момент времени

  // float omg = whl_vel_des*radius/R;
  
  // float dqL = 0.0; // скорость вращения левого колеса
  // float dqR = 0.0; // скорость вращения правого колеса

  // float qL_des; // желаемый угол поворота левого колеса [об]
  // float qR_des;

  // float dqL_des = omg*(R - L/2.0)/radius; // желаемая скорость вращения левого колеса [об/мин]
  // float dqR_des = omg*(R + L/2.0)/radius; 

  // Serial.println("dqL_des: " + String(dqL_des, 3) + " dqR_des: " + String(dqR_des, 3));

  // // ошибки
  // float qL_err = 0.0;
  // float qR_err = 0.0;
  // //float dqL_err = 0.0;
  // //float dqR_err = 0.0;

  // float uL = 0.0;
  // float uR = 0.0;

  // // скоррекированные значения [об/мин]
  // float whl_velL;
  // float whl_velR;

  // float dt = 0.0; // измерительный промежуток [мин]

  // uint32_t start;

  // while(true)
  // {
  //   switch(getSerialData())
  //   {
  //     case('w'):
  //       isReady = true;
  //       isMoving = true;
  //       start = millis();
  //       //dq_des = whl_vel_des;
  //       goForward(dqL_des, dqR_des);
  //       break;

  //     // case('x'):
  //     //   isReady = true;
  //     //   isMoving = true;
  //     //   start = millis();
  //     //   //dq_des = -whl_vel_des;
  //     //   goForward(-dqL_des, -dqR_des);
  //     //   break;

  //     case('s'):
  //       stopMoving();
  //       isReady = false;
  //       isMoving = false;
  //       break;

  //     default:
  //       break;
  //   }

  //   if(isMoving && isReady)
  //   {
  //     // углы [об]
  //     qL_curr = motorBlockL->getRotAngle();
  //     qR_curr = motorBlockR->getRotAngle();

  //     t_curr = millis() - start;

  //     //q_des = dq_des * t_curr / 60000.0; // желаемый угол [об]
      
  //     qL_des = dqL_des * t_curr / 60000.0;
  //     qR_des = dqR_des * t_curr / 60000.0;

  //     dt = (t_curr - t_prev) / 60000.0; // промежуток между замерами [мин]

  //     // оценка измеренной скорости вращения колёс [об/мин]
  //     dqL = (qL_curr - qL_prev) / dt;
  //     dqR = (qR_curr - qR_prev) / dt;

  //     qL_err = qL_des - qL_curr;
  //     qR_err = qR_des - qR_curr;

  //     //dqL_err = dq_des - dqL;
  //     //dqR_err = dq_des - dqR;

  //     String msg_q_err = "qL_err: " + String(qL_err, 3) + " qR_err: " + String(qR_err, 3) + " dt: " + String(dt, 4);
  //     Serial.println(msg_q_err); 

  //     //uL = pidL->computeControl(qL_err, dt);
  //     //uR = pidR->computeControl(qR_err, dt);

  //     whl_velL = dqL_des + uL;
  //     whl_velR = dqR_des + uR;

  //     if(deb)
  //     {
  //       String msg_u = "uL: " + String(uL, 3) + " uR: " + String(uR, 3) + " whl_velL: " + String(whl_velL, 3) + " whl_velR: " + String(whl_velR, 3);
  //       Serial.println(msg_u);

  //       // int pwmL = map(abs(whl_velL), 0, 150, 0, 255);
  //       // int pwmR = map(abs(whl_velR), 0, 150, 0, 255);
  //       // int pwm_des = map(abs(dq_des), 0, 150, 0, 255);
  //       // String msg_pwm = "PWM L: " + String(pwmL) + " PWM R: " + String(pwmR) + " Desired PWM: " + String(pwm_des);
  //       // Serial.println(msg_pwm);
  //     } 
      
  //     else 
  //     {
  //       goForward(whl_velL, whl_velR);
  //     }

  //     // if(abs(q_des) >= 9.975)
  //     // {
  //     //   Serial.println("Stopping");
  //     //   stopMoving();

  //     //   qL_curr = motorBlockL->getRotAngle();
  //     //   qR_curr = motorBlockR->getRotAngle();
  //     //   t_curr = millis() - start;
  //     //   String msg_ang = "L: " + String(qL_curr, 3) + " R: " + String(qR_curr, 3) + " Time: " + String(t_curr);
  //     //   Serial.println(msg_ang);

  //     //   isMoving = false;
  //     //   isReady = false;
  //     // }

  //     qL_prev = qL_curr;
  //     qR_prev = qR_curr;
  //     t_prev = t_curr;

  //   }

  //   delay(del);
  // }
// }


// ##########################################################


// ==== manual control ==== //
void TwoWheeledRobot::manualControl(int del)
{
  int vel = 60;

  float r = getRadiusWheels();
  float L = baseLength;

  float deltaAngL;
  float deltaAngR;

  float distWheelL = 0.0;
  float distWheelR = 0.0;
  float distWheelC = 0.0;

  while(!globalStop)
  {
    switch (getSerialData())
    {
      case ('z'):
        globalStop = true;
        stopMoving();
        return;
      break;
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

    distWheelL = distWheelL + deltaAngL*r;
    distWheelR = distWheelR + deltaAngR*r;
    distWheelC = (distWheelR + distWheelL) / 2;

    pos.estCurrentPosition(deltaAngL, deltaAngR, r, L);

    // Serial.println(distWheelC);

    // if(distWheelC >= 1.0) {
    //   Serial.println("1 m traveled, distance: " + String(distWheelC, 3));
    //   stopMoving();
    //   break;
    // }

    String msg_enc = String(pos.x, 3) + " " + String(pos.y, 3) + " " + String(pos.theta);
    Serial.println(msg_enc);

    delay(del);
  }
}

void TwoWheeledRobot::stopMoving()
{
  motorBlockL->stopMoving();
  motorBlockR->stopMoving();
}

void TwoWheeledRobot::goForward(int velL, int velR)
{
  motorBlockL->setVelocity(velL, vel.maxWheel, newMinRange);
  motorBlockR->setVelocity(velR, vel.maxWheel, newMinRange);
}

void TwoWheeledRobot::turnLeft(int velL, int velR)
{
  motorBlockL->setVelocity(velL, vel.maxWheel, newMinRange);
  motorBlockR->setVelocity(velR, vel.maxWheel, newMinRange);
}

void TwoWheeledRobot::turnRight(int velL, int velR)
{
  motorBlockL->setVelocity(velL, vel.maxWheel, newMinRange);
  motorBlockR->setVelocity(velR, vel.maxWheel, newMinRange);
}

int TwoWheeledRobot::checkCurrent(byte PIN_CURRENT_SENSOR)
{
  return analogRead(PIN_CURRENT_SENSOR);
}

int TwoWheeledRobot::linear2angular(int vel)
{
  return vel / (2.0*PI*WHEEL_RADIUS);
}