#include "twoWheeledRobot.h"
#include "constants.h"

//#include <SPI.h>
//#include <MFRC522.h>

#define RST_PIN         8          // Configurable, see typical pin layout above
#define SS_PIN          53         // Configurable, see typical pin layout above

TwoWheeledRobot::TwoWheeledRobot()
  :reachedGoal(false), globalStop(false),
  PIN_CURRENT_SENSOR(A12),
  inByte(0), newMinRange(0)
{
  // RFID READER
  //MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance // ????????????????????????????????????
  if(!Serial) 
    Serial.begin(9600);

  rfidReader = new RFIDReader(SS_PIN, RST_PIN);
  rfidReader->readerStart();

  motorBlockL = new MotorBlock();
  motorBlockR = new MotorBlock();
  pidL = new PID();
  pidR = new PID();
  pid = new PID();
  pinMode(PIN_CURRENT_SENSOR, LOW);
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
  vel.maxRobot = maxVel * 2.0 * 3.141593 * wheelRadius / 60.0; // макс. линейная скорость робота [м/с]
  // if (DEBUG){
  //   Serial.print("vel.max: "); Serial.println(vel.maxWheel);
  // }
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

// void TwoWheeledRobot::sendSerialData(std::sring &str)
// {
//   Serial.writeln(str);
// }


void TwoWheeledRobot::serialControl(bool deb) {
  Serial.println(" ===== Choose mode ===== ");
  while (true)
  {
    inByte = getSerialData();
    switch (inByte)
      {
        case ('m'):
          Serial.println("=== You are using manual control ===");
          manualControl(50);
          break;

        case ('g'):
          Serial.println("========= GO GO GO =========");
          goToGoal(0.6, 0.6, true, 50, deb, true, 1);
          break;
        
        case ('c'):
          Serial.println("====== Circle trajectory ======");
          goCircle(0.6, 4, deb);
          break;

        // case ('t'):
        //   Serial.println(" ===== Rotation test ===== ");
        //   rot_test(90, 50, deb);
        //   break;

        case ('r'):
          Serial.println(" ===== RFID reader test ===== ");
          rfidTest(50);
          break;
      }
  }
}

void TwoWheeledRobot::rfidTest(int del) {
  int rfidFound;

  while(true) {
    rfidFound = rfidReader->checkReaderData();
    if(rfidFound > 0)
      Serial.println(rfidFound);

    delay(del);
  }
}

void TwoWheeledRobot::goCircle(float radius, int ptsNum, bool deb)
{
  float x0 = 0.0;
  float y0 = 0.0;

  float x;
  float y;
  
  //bool followRFID = false;
  bool followRFID = true;

  bool isFinish = false;

  float dPhi = 2.0*3.141593 / ptsNum;

  //long t_start = millis();

  for(int i=1; i <= ptsNum; i++)
  {
    //if(i % 2 == 0) followRFID = true;
    if(i == ptsNum) { isFinish = true; }
    x = x0 + radius * sin(dPhi*i);
    y = (y0 + radius) - radius * cos(dPhi*i);
    Serial.println("X" + String(i) + ": " + String(x, 3) + " Y" + String(i) + ": " + String(y, 3));
    goToGoal(x, y, isFinish, 50, deb, followRFID, i);
    if(globalStop) 
    { 
      Serial.println(" ==== GLOBAL STOP ==== ");
      break; 
    }
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
void TwoWheeledRobot::goToGoal(float xGoal, float yGoal, bool isFinish, int del, bool deb, bool followRFID, int idRFID) {


  reachedGoal = false;

  // поворот колёс за время между оценкой положения робота
  float deltaAngL;
  float deltaAngR;

  // скорректированные скорости колёс
  float velL;
  float velR;

  //Расчет угла, на котором расположена целевая точка
  pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);

  // if (DEBUG){
  //   Serial.print("pos.thetaGoal: "); Serial.println(pos.thetaGoal); // ----- TEST
  // }

  float r = getRadiusWheels();
  float L = baseLength;
  float err = 0.0;

  float dt = 0.0;

  int rfidFound = 0;

  while(!reachedGoal && !globalStop) {
    //Расчет угла, на котором расположена целевая точка
    //pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);
    //Serial.println("Theta goal: " + String(pos.thetaGoal, 3) + " Theta: " + String(pos.theta, 3));

    //t_curr = millis() - t_start;
    //dt = (t_curr - t_prev) / 1000.0;

    dt = (del + 25) / 1000.0; // типа плюс время на вычисления
    //Serial.println(dt);
    err = pid->computeAngleError(pos.thetaGoal, pos.theta);
    Serial.println("Err theta: " + String(err, 3));

    vel.ang = pid->computeControl(err, dt, rfidFound);
    vel.lin = vel.computeLinearSpeed(err);

    // String msg_vel = "Ang_Vel: " + String(vel.ang, 3);
    // Serial.println(msg_vel);

    //Расчет скоростей для каждого двигателя в об/мин
    velL = ((2.0 * vel.lin - vel.ang * L) / (2.0 * r)) * 60.0 / (2*3.141593);
    velR = ((2.0 * vel.lin + vel.ang * L) / (2.0 * r)) * 60.0 / (2*3.141593);

    //Serial.println("vleL: " + String(velL, 3) + " velR " + String(velR, 3));

    // motorBlockL->setVelocity(velL, vel.maxWheel, newMinRange);
    // motorBlockR->setVelocity(velR, vel.maxWheel, newMinRange);

    if(!deb)
      goForward(velL, velR);

    //float distWheelL = motorBlockL->getTraveledDistance();
    //float distWheelR = motorBlockR->getTraveledDistance();
    //float distWheelC = (distWheelR+distWheelL) / 2;
    //pos.computeCurentPose(distWheelL, distWheelR, distWheelC, L);

    deltaAngL = motorBlockL->getDeltaAngle();
    deltaAngR = motorBlockR->getDeltaAngle();
    pos.estCurrentPosition(deltaAngL, deltaAngR, r, L);

    String msg_pos = "X: " + String(pos.x, 3) + " Y: " + String(pos.y, 3) + " Th: " + String(pos.theta, 3);
    Serial.println(msg_pos);

    if(!followRFID) {
      if((abs(xGoal-pos.x) < 0.05) && (abs(yGoal-pos.y) < 0.05))
      {
        Serial.println("PT REACHED");
        Serial.println("X_e: " + String(xGoal-pos.x, 3) + " Y_e: " + String(yGoal-pos.y, 3) + " Theta: " + String(pos.theta, 3));
        reachedGoal = true;
      }
    } else {
      rfidFound = rfidReader->checkReaderData();
      // if(rfidFound == 1) {
      //   reachedGoal = true;
      //   Serial.println(rfidFound);
      // }
      switch (rfidFound) {
        case 0:
          break;

        Serial.println("X_err: " + String(xGoal-pos.x, 3) + " Y_err: " + String(yGoal-pos.y));
        case 1:
          //pos.x = 0.6;
          //pos.y = 0.6;
          Serial.println("RFID 1 REACHED: 0.6 0.6");
          break;
        case 2:
          pos.x = 0.0;
          pos.y = 1.2;
          Serial.println("RFID 2 REACHED 0.0 1.2");
          break;
        case 3:
          pos.x = -0.6;
          pos.y = 0.6;
          Serial.println("RFID 3 REACHED -0.6 0.6");
          break;

        case 4:
          pos.x = 0.0;
          pos.y = 0.0;
          Serial.println("BACK TO BASE 0.0 0.0");
          reachedGoal = true;
          break;

        default:
          Serial.println("Stranger");
          break;
      }

      if(rfidFound == idRFID) {
        reachedGoal = true;
        //break;
      }
    }

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
    
    // if (DEBUG_PLOT){
    //   Serial.print("$");
    //   Serial.print(pos.x, 3);Serial.print(" ");Serial.print(pos.y, 3);
    //   Serial.println(";");
    // }
    // if (DEBUG){
    //   Serial.print("err: "); Serial.println(err, 3);
    // }
    // if (DEBUG){
    //   Serial.print("$"); Serial.print(err); Serial.println(";");
    // }
    // if (DEBUG){
    //   Serial.print("angVel: "); Serial.print(vel.ang);
    //   Serial.print("  linVel: "); Serial.println(vel.lin);
    // }
    // if (DEBUG){
    //   Serial.print("velL: "); Serial.print(velL);
    //   Serial.print("  velR: "); Serial.println(velR);
    // }
    // if (DEBUG){
    //   Serial.print("distWheelL: "); Serial.print(distWheelL, 3);
    //   Serial.print("  distWheelR: "); Serial.print(distWheelR, 3);
    //   Serial.print("  distWheelC: "); Serial.println(distWheelC, 3);
    // }
    // if (DEBUG){
    //   Serial.print("X: "); Serial.print(pos.x, 3);
    //   Serial.print("  Y: "); Serial.print(pos.y, 3);
    //   Serial.print("  Th: "); Serial.println(pos.theta, 3);
    //   Serial.println("  -------  ");
    // }
    
    // Serial.println(checkCurrent(PIN_CURRENT_SENSOR));
    // Serial.println(i);
    
    // if (i>7){
    //   if(checkCurrent(PIN_CURRENT_SENSOR)>550)
    //   {
    //     stopMoving();
    //     break;
    //   }
    // }

  
    switch(getSerialData())
    {
      case('s'):
        stopMoving();
        globalStop = true;
      break;

      case('r'):
        stopMoving();
        break;
      break;
    }

    delay(del);
  }
}

// ############## Прямолинейное движение в пид-контроллером углов поворота колёс ###############
void TwoWheeledRobot::rot_test(int whl_vel_des, byte del, bool deb) {

  float radius = getRadiusWheels();
  float R = 0.5;
  float L = baseLength;

  Serial.println("radius: " + String(radius, 3) + " R: " + String(R, 2) + " L: " + String(L, 3));

  bool isReady = false;
  bool isMoving = false;

  float qL_curr = 0.0; // текущий измеренный угол поворота левого колеса
  float qR_curr = 0.0; // текущий измеренный угол повората правого колеса
  int t_curr = 0; // текущий момент времени

  float qL_prev = 0.0; // предыдущий измеренный угол поворота левого колеса
  float qR_prev = 0.0; // предыдущий измеренный угол поворота правого колеса
  int t_prev = 0; // предыдущий момент времени

  float omg = whl_vel_des*radius/R;
  
  float dqL = 0.0; // скорость вращения левого колеса
  float dqR = 0.0; // скорость вращения правого колеса

  float qL_des; // желаемый угол поворота левого колеса [об]
  float qR_des;

  float dqL_des = omg*(R - L/2.0)/radius; // желаемая скорость вращения левого колеса [об/мин]
  float dqR_des = omg*(R + L/2.0)/radius; 

  Serial.println("dqL_des: " + String(dqL_des, 3) + " dqR_des: " + String(dqR_des, 3));

  // ошибки
  float qL_err = 0.0;
  float qR_err = 0.0;
  //float dqL_err = 0.0;
  //float dqR_err = 0.0;

  float uL = 0.0;
  float uR = 0.0;

  // скоррекированные значения [об/мин]
  float whl_velL;
  float whl_velR;

  float dt = 0.0; // измерительный промежуток [мин]

  uint32_t start;

  while(true)
  {
    switch(getSerialData())
    {
      case('w'):
        isReady = true;
        isMoving = true;
        start = millis();
        //dq_des = whl_vel_des;
        goForward(dqL_des, dqR_des);
        break;

      // case('x'):
      //   isReady = true;
      //   isMoving = true;
      //   start = millis();
      //   //dq_des = -whl_vel_des;
      //   goForward(-dqL_des, -dqR_des);
      //   break;

      case('s'):
        stopMoving();
        isReady = false;
        isMoving = false;
        break;

      default:
        break;
    }

    if(isMoving && isReady)
    {
      // углы [об]
      qL_curr = motorBlockL->getRotAngle();
      qR_curr = motorBlockR->getRotAngle();

      t_curr = millis() - start;

      //q_des = dq_des * t_curr / 60000.0; // желаемый угол [об]
      
      qL_des = dqL_des * t_curr / 60000.0;
      qR_des = dqR_des * t_curr / 60000.0;

      dt = (t_curr - t_prev) / 60000.0; // промежуток между замерами [мин]

      // оценка измеренной скорости вращения колёс [об/мин]
      dqL = (qL_curr - qL_prev) / dt;
      dqR = (qR_curr - qR_prev) / dt;

      qL_err = qL_des - qL_curr;
      qR_err = qR_des - qR_curr;

      //dqL_err = dq_des - dqL;
      //dqR_err = dq_des - dqR;

      String msg_q_err = "qL_err: " + String(qL_err, 3) + " qR_err: " + String(qR_err, 3) + " dt: " + String(dt, 4);
      Serial.println(msg_q_err); 

      //uL = pidL->computeControl(qL_err, dt);
      //uR = pidR->computeControl(qR_err, dt);

      whl_velL = dqL_des + uL;
      whl_velR = dqR_des + uR;

      if(deb)
      {
        String msg_u = "uL: " + String(uL, 3) + " uR: " + String(uR, 3) + " whl_velL: " + String(whl_velL, 3) + " whl_velR: " + String(whl_velR, 3);
        Serial.println(msg_u);

        // int pwmL = map(abs(whl_velL), 0, 150, 0, 255);
        // int pwmR = map(abs(whl_velR), 0, 150, 0, 255);
        // int pwm_des = map(abs(dq_des), 0, 150, 0, 255);
        // String msg_pwm = "PWM L: " + String(pwmL) + " PWM R: " + String(pwmR) + " Desired PWM: " + String(pwm_des);
        // Serial.println(msg_pwm);
      } 
      
      else 
      {
        goForward(whl_velL, whl_velR);
      }

      // if(abs(q_des) >= 9.975)
      // {
      //   Serial.println("Stopping");
      //   stopMoving();

      //   qL_curr = motorBlockL->getRotAngle();
      //   qR_curr = motorBlockR->getRotAngle();
      //   t_curr = millis() - start;
      //   String msg_ang = "L: " + String(qL_curr, 3) + " R: " + String(qR_curr, 3) + " Time: " + String(t_curr);
      //   Serial.println(msg_ang);

      //   isMoving = false;
      //   isReady = false;
      // }

      qL_prev = qL_curr;
      qR_prev = qR_curr;
      t_prev = t_curr;

    }

    delay(del);
  }
}


// ##########################################################


// ==== manual control ==== //
void TwoWheeledRobot::manualControl(int del)
{
  int vel = 60;

  float r = getRadiusWheels();
  float L = baseLength;

  float deltaAngL;
  float deltaAngR;

  while(true)
  {
    switch (getSerialData())
    {
      case ('w'):
        goForward(vel, vel);
      break;
      case ('x'):
        goForward(-vel, -vel);
      break;
      case ('s'):
        stopMoving();
      break;
      case ('d'):
        turnRight(vel, -vel);
      break;
      case ('a'):
        turnLeft(-vel, vel);
      break;
      case ('e'):
        // turnRight(100, -50);
        vel += 5;
        (vel >= 150) ? 150 : vel;

      break;
      case ('q'):
        // turnLeft(-50, 100);
        vel -= 5;
        (vel <= 0) ? 0 : vel;
      break;
    }

    // float distWheelL = motorBlockL->getTraveledDistance();
    // float distWheelR = motorBlockR->getTraveledDistance();
    // float distWheelC = (distWheelR + distWheelL) / 2;
    // pos.computeCurentPose(distWheelL, distWheelR, distWheelC, baseLength);


    deltaAngL = motorBlockL->getDeltaAngle();
    deltaAngR = motorBlockR->getDeltaAngle();
    pos.estCurrentPosition(deltaAngL, deltaAngR, r, L);


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
  motorBlockL->setVelocity(velL, vel.maxWheel, 0);
  motorBlockR->setVelocity(velR, vel.maxWheel, 0);
}

void TwoWheeledRobot::turnLeft(int velL, int velR)
{
  motorBlockL->setVelocity(velL, vel.maxWheel, 0);
  motorBlockR->setVelocity(velR, vel.maxWheel, 0);
}

void TwoWheeledRobot::turnRight(int velL, int velR)
{
  motorBlockL->setVelocity(velL, vel.maxWheel, 0);
  motorBlockR->setVelocity(velR, vel.maxWheel, 0);
}

int TwoWheeledRobot::checkCurrent(byte PIN_CURRENT_SENSOR)
{
  return analogRead(PIN_CURRENT_SENSOR);
}
