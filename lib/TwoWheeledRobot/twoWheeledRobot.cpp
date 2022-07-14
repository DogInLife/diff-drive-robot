#include "twoWheeledRobot.h"
#include "constants.h"

TwoWheeledRobot::TwoWheeledRobot()
  :reachedGoal(0), 
  PIN_CURRENT_SENSOR(A12),
  inByte(0), newMinRahge(150)
{
  Serial.begin(9600);
  motorBlockL = new MotorBlock();
  motorBlockR = new MotorBlock();
  pid = new PID();
  pinMode(PIN_CURRENT_SENSOR, LOW);
}

TwoWheeledRobot::~TwoWheeledRobot()
{
  delete motorBlockL;
  delete motorBlockR;
  delete pid;
}

void TwoWheeledRobot::createWheels(float wheelRadius, float baseLength, float maxVel)
{
  motorBlockL->createWheel(wheelRadius);
  motorBlockR->createWheel(wheelRadius);
  this->baseLength = baseLength;
  // vel.max = 6.28/60*wheelRadius*maxVel;
  vel.maxWheel = maxVel;
  vel.maxRobot = maxVel * wheelRadius;
  if (DEBUG){
    Serial.print("vel.max: "); Serial.println(vel.maxWheel);
  }
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


void TwoWheeledRobot::serialControl()
{
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
          goToGoal(1,1, 50);
        break;
      }
  }
}


// ====================== robot behavior ===================== //
// ======= GO ======== //
void TwoWheeledRobot::goToGoal(float xGoal, float yGoal, float dt)
{
  //Расчет угла, на котором расположена целевая точка
  pos.thetaGoal = atan2(yGoal-pos.y, xGoal-pos.x);
  if (DEBUG){
    Serial.print("pos.thetaGoal: "); Serial.println(pos.thetaGoal); // ----- TEST
  }

  float R = getRadiusWheels();
  float L = baseLength;
  float err = 0;
  

  while(!reachedGoal)
  {
    err = pid->computeAngleError(pos.thetaGoal, pos.theta);
    
    vel.ang = pid->computeControl(err, dt/1000);
    vel.lin = vel.computeLinearSpeed();


    //Расчет скоростей для каждого двигателя
    float velR = (2*vel.lin + vel.ang*L)/(2*R);
    float velL = (2*vel.lin - vel.ang*L)/(2*R);

    motorBlockL->setVelocity(velL, vel.maxWheel, newMinRahge);
    motorBlockR->setVelocity(velR, vel.maxWheel, newMinRahge);

    float distWheelL = motorBlockL->getTraveledDistance();
    float distWheelR = motorBlockR->getTraveledDistance();
    float distWheelC = (distWheelR+distWheelL) / 2;

    pos.computeCurentPose(distWheelL, distWheelR, distWheelC, L);
 

    if((abs(pos.x-xGoal) < 0.03) && (abs(pos.y-yGoal) < 0.03))
    {
      Serial.println("You have reached your goal");
      Serial.print("err_X: "); Serial.print(pos.x-xGoal, 3);
      Serial.print("  err_Y: "); Serial.println(pos.y-yGoal, 3);
      reachedGoal = true;
    }


    if(reachedGoal)
    {
      stopMoving();
      break;
    }
    
    if (DEBUG_PLOT){
      Serial.print("$");
      Serial.print(pos.x, 3);Serial.print(" ");Serial.print(pos.y, 3);
      Serial.println(";");
    }
    if (DEBUG){
      Serial.print("err: "); Serial.println(err, 3);
    }
    if (DEBUG){
      Serial.print("$"); Serial.print(err); Serial.println(";");
    }
    if (DEBUG){
      Serial.print("angVel: "); Serial.print(vel.ang);
      Serial.print("  linVel: "); Serial.println(vel.lin);
    }
    if (DEBUG){
      Serial.print("velL: "); Serial.print(velL);
      Serial.print("  velR: "); Serial.println(velR);
    }
    if (DEBUG){
      Serial.print("distWheelL: "); Serial.print(distWheelL, 3);
      Serial.print("  distWheelR: "); Serial.print(distWheelR, 3);
      Serial.print("  distWheelC: "); Serial.println(distWheelC, 3);
    }
    if (DEBUG){
      Serial.print("X: "); Serial.print(pos.x, 3);
      Serial.print("  Y: "); Serial.print(pos.y, 3);
      Serial.print("  Th: "); Serial.println(pos.theta, 3);
      Serial.println("  -------  ");
    }
    
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
      break;
      case('r'):
        stopMoving();
        break;
      break;
    }

    delay(dt);
  }
}

// ############## Вывод углов поворота колёс ###############
void TwoWheeledRobot::rot_test(int vel, byte dt)
{
  bool isReady = false;
  bool isMoving = false;
  //int vel = 15;

  float qL_curr = 0.0; // текущий измеренный угол поворота левого колеса
  float qR_curr = 0.0; // текущий измеренный угол повората правого колеса
  int t_curr = 0; // текущий момент времени

  float qL_prev = 0.0; // предыдущий измеренный угол поворота левого колеса
  float qR_prev = 0.0; // предыдущий измеренный угол поворота правого колеса
  int t_prev = 0; // предыдущий момент времени

  float dqL = 0.0; // скорость вращения левого колеса
  float dqR = 0.0; // скорость вращения правого колеса

  float q_des; // желаемый угол поворота колеса [град]
  float dq_des = vel * 6.0; // желаемая скорость вращения колеса [град / сек]

  // ошибки
  float qL_err = 0.0;
  float qR_err = 0.0;
  float dqL_err = 0.0;
  float dqR_err = 0.0;

  uint32_t start;

  while(true)
  {
    switch(getSerialData())
    {
      case('w'):
        // isStopped = false;
        isReady = true;
        isMoving = true;
        start = millis();
        goForward(vel, vel);
        break;

      case('x'):
        // isStopped = false;
        isReady = true;
        isMoving = true;
        start = millis();
        goForward(-vel, -vel);
        break;

      case('s'):
        stopMoving();
        //isStopped = true;
        isReady = false;
        isMoving = false;
        break;

      default:
        break;
    }

    if(isMoving && isReady)
    {
      qL_curr = motorBlockL->getRotAngle();
      qR_curr = motorBlockR->getRotAngle();
      t_curr = millis() - start;

      q_des = dq_des * t_curr / 1000.0;
      
      qL_err = q_des - qL_curr;
      qR_err = q_des - qR_curr;
      // String msg_ang = "L: " + String(rotAngleL_curr, 3) + " R: " + String(rotAngleR_curr, 3) + " Time: " + String(t_curr) + " Desired angle: " + String(q_des, 3);
      // Serial.println(msg_ang);
      
      dqL = (qL_curr - qL_prev) * 1000 / (t_curr - t_prev);
      dqR = (qR_curr - qR_prev) * 1000 / (t_curr - t_prev);
      // String msg_vel = "Vel L: " + String(dqL, 3) + " Vel R: " + String(dqR, 3) + " Desired velocity: " + String(rotVel_des, 3);
      // Serial.println(msg_vel);
      dqL_err = dq_des - dqL;
      dqR_err = dq_des - dqR;

      String msg_err = "qL: " + String(qL_err, 3) + " qR: " + String(qR_err, 3) + " ==//== dqL: " + String(dqL_err, 3) + " dqR: " + String(dqR_err, 3);
      Serial.println(msg_err);

      if(q_des >= 720.0)
      {
        Serial.println("Stopping");
        stopMoving();

        qL_curr = motorBlockL->getRotAngle();
        qR_curr = motorBlockR->getRotAngle();
        t_curr = millis() - start;
        String msg_ang = "L: " + String(qL_curr, 3) + " R: " + String(qR_curr, 3) + " Time: " + String(t_curr);
        Serial.println(msg_ang);

        isMoving = false;
        isReady = false;
      }

      qL_prev = qL_curr;
      qR_prev = qR_curr;
      t_prev = t_curr;

    }

    delay(dt);
  }
}


// ##########################################################


// ==== manual control ==== //
void TwoWheeledRobot::manualControl(float dt)
{
  int vel = 30;

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
        turnRight(vel, 0);
      break;
      case ('a'):
        turnLeft(0, vel);
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

    float distWheelL = motorBlockL->getTraveledDistance();
    float distWheelR = motorBlockR->getTraveledDistance();
    float distWheelC = (distWheelR + distWheelL) / 2;

    pos.computeCurentPose(distWheelL, distWheelR, distWheelC, baseLength);

    String msg_enc = String(pos.x, 3) + " " + String(pos.y, 3);
    Serial.println(msg_enc);
    delay(dt);
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
