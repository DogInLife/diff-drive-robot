#include "twoWheeledRobot.h"
#include "constants.h"


TwoWheeledRobot::TwoWheeledRobot() 
{
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

float TwoWheeledRobot::getRadiusWheels()
{
  return motorBlockL->getRadiusWheels();
}


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
  
  // ================================== FOR ===============================
  for (int i = 0; i <= 50; i++) 
  {
    err = pid->computeAngleError(pos.thetaGoal, pos.theta);
    
    if(DEBUG){
      Serial.print("err: "); Serial.println(err, 3);
    }
    if(DEBUG){
      Serial.print("$"); Serial.print(err); Serial.println(";");
    }

    vel.ang = pid->computeControl(err, dt/1000);
    vel.lin = vel.computeLinearSpeed();
    if (DEBUG){
      Serial.print("angVel: "); Serial.print(vel.ang);
      Serial.print("  linVel: "); Serial.println(vel.lin);
    }

    //Расчет скоростей для каждого двигателя
    float velR = (2*vel.lin + vel.ang*L)/(2*R);
    float velL = (2*vel.lin - vel.ang*L)/(2*R);
    if (DEBUG){
      Serial.print("velL: "); Serial.print(velL);
      Serial.print("  velR: "); Serial.println(velR);
    }

    // motorBlockL->setVelocity(0, vel.maxRobot/R);
    // motorBlockR->setVelocity(0, vel.maxRobot/R);

    motorBlockL->setVelocity(velL, vel.maxWheel);
    motorBlockR->setVelocity(velR, vel.maxWheel);


    float distWheelL = motorBlockL->getTraveledDistance();
    float distWheelR = motorBlockR->getTraveledDistance();
    float distWheelC = (distWheelR+distWheelL) / 2;
    if (DEBUG){
      Serial.print("distWheelL: "); Serial.print(distWheelL, 3);
      Serial.print("  distWheelR: "); Serial.print(distWheelR, 3);
      Serial.print("  distWheelC: "); Serial.println(distWheelC, 3);
    }

    pos.computeCurentPose(distWheelL, distWheelR, distWheelC, L);
    if (DEBUG){
      Serial.print("X: "); Serial.print(pos.x, 3);
      Serial.print("  Y: "); Serial.print(pos.y, 3);
      Serial.print("  Th: "); Serial.println(pos.theta, 3);
      Serial.println("  -------  ");
    }
    if (DEBUG_PLOT){
      Serial.print("$");
      Serial.print(pos.x, 3);Serial.print(" ");Serial.print(pos.y, 3);
      Serial.println(";");
    }


    if((abs(pos.x-xGoal) < 0.03) && (abs(pos.y-yGoal) < 0.03))
    {
      Serial.println("You have reached your goal");
      Serial.print("err_X: "); Serial.print(pos.x-xGoal, 3);
      Serial.print("  err_Y: "); Serial.println(pos.y-yGoal, 3);
      stopMoving();
      break;
    }

    // Serial.println(checkCurrent(PIN_CURRENT_SENSOR));
    // Serial.println(i);
    if (i>7){
      if(checkCurrent(PIN_CURRENT_SENSOR)>550)
      {
        stopMoving();
        break;
      }
    }

    delay(dt);
  }
  stopMoving();
  Serial.println(" === STOP === ");
}

int TwoWheeledRobot::checkCurrent(byte PIN_CURRENT_SENSOR)
{
  return analogRead(PIN_CURRENT_SENSOR);
}

void TwoWheeledRobot::stopMoving()
{
  motorBlockL->stopMoving();
  motorBlockR->stopMoving();
}


void TwoWheeledRobot::goForward()
{
  motorBlockL->setVelocity(150, vel.maxWheel);
  motorBlockR->setVelocity(150, vel.maxWheel);
  // float distWheelL = motorBlockL->getTraveledDistance();
  // float distWheelR = motorBlockR->getTraveledDistance();
}

void TwoWheeledRobot::turnLeft()
{
  // motorBlockL->setVelocity(150, vel.maxWheel);
  motorBlockL->setVelocity(50, vel.maxWheel);
  for (int i = 0; i <= 50; i++) 
  {
    float distWheelL = motorBlockL->getTraveledDistance();
    Serial.println(distWheelL);
    delay(50);
  }
}

void TwoWheeledRobot::turnRight()
{
  // motorBlockL->setVelocity(150, vel.maxWheel);
  motorBlockR->setVelocity(50, vel.maxWheel);
  for (int i = 0; i <= 50; i++) 
  {
    float distWheelR = motorBlockR->getTraveledDistance();
    Serial.println(distWheelR);
    delay(50);
  }

}
