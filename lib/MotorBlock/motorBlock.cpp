#include "motorBlock.h"

MotorBlock::MotorBlock(float wheelRadius, float maxVel, byte encPin, byte driverPin1, byte driverPin2, byte driverPinPWM) {
    setWheelRadius(wheelRadius);
    setMaxVel(maxVel);
    encoder = new Encoder();
    setEncorerPin(encPin);
    setDriverPin(driverPin1, driverPin2, driverPinPWM);
    speedPID = new PID(Kp, Ki, Kd);
}
MotorBlock::~MotorBlock()
{
    delete encoder;
    delete speedPID;
}

void MotorBlock::setCoefficientPID(float Kp, float Ki, float Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    speedPID->setCoefficient(Kp, Ki, Kd);
    speedPID->resetErr();
}
void MotorBlock::setDeltaPID(float dt)              { del = dt; }
void MotorBlock::setWheelRadius(float wheelRadius)  { this->wheelRadius = wheelRadius; }
void MotorBlock::setMaxVel(float maxVel)            { this->maxVel = maxVel; }
void MotorBlock::setEncorerPin(byte encPin)         { encoder->setPin(encPin); }
void MotorBlock::setDriverPin(byte driverPin1, byte driverPin2, byte driverPinPWM)
{
    IN_DRIVER_PIN_1 = driverPin1;
    IN_DRIVER_PIN_2 = driverPin2;
    PWM_PIN = driverPinPWM;
    pinMode(IN_DRIVER_PIN_1,  OUTPUT);
    pinMode(IN_DRIVER_PIN_2,  OUTPUT);
}

void MotorBlock::tickPrepare() {
    lastTime = millis();
    lastSpeed = 0.0f;
}
void MotorBlock::tick() {
    float deltaAngle = getDeltaAngle();
    angleOffset += deltaAngle;

    unsigned long time = millis();
    float deltaTime = (float)(time - lastTime) / 1000.0f;
    speedSum += deltaAngle / deltaTime;
    lastTime = time;
    speedCount++;
}
void MotorBlock::updatePWM() {
    updatePWM(del);
}
void MotorBlock::updatePWM(float dt) {
    float errSpeed = speedPID->updateErr(targetSpeed - getWheelSpeed(), dt);

    pwm = map(constrain(abs(errSpeed), 0, maxVel), 0, maxVel, 0, 255);
    if (abs(pwm) < minPWM) 
    {
        pwm = 0;
    } 
    else 
    {
        if (errSpeed > 0)
        {
            digitalWrite(IN_DRIVER_PIN_1, LOW);
            digitalWrite(IN_DRIVER_PIN_2, HIGH);
        } 
        else 
        {
            digitalWrite(IN_DRIVER_PIN_1, HIGH);
            digitalWrite(IN_DRIVER_PIN_2, LOW);
        }
    }
    analogWrite(PWM_PIN, pwm);
}
void MotorBlock::resetErrPID() {
    speedPID->resetErr();
}

void MotorBlock::stopMoving()
{   
    for (int i = pwm; i != 0 ;)
    {
        i=i-10;
        i=(i<0)?0:i;
        analogWrite(PWM_PIN, i);
    }
    targetSpeed = 0;
}
void MotorBlock::updateVelocity(float vel)
{
    // PID добавить!!!!
    targetSpeed = vel;
}

float MotorBlock::getWheelSpeed()
{
    if (speedCount == 0) return lastSpeed;
    lastSpeed = speedSum/speedCount;
    speedSum = 0;
    speedCount = 0;
    return lastSpeed;
}
float MotorBlock::getDeltaAngle()
{
    float ovAng_prev = encoder->overallTurnEnc_k0; // число пройденных делений в предыдущий момент
    float ovAng_curr = encoder->getOverallTurn(); // число пройденных делений в текущий момент
    float deltaTurn = (ovAng_curr - ovAng_prev) / 4095.0; // переход от делений к оборотам
    encoder->overallTurnEnc_k0 = ovAng_curr;

    if (deltaTurn > 300000000) return 0;          // Выброс?!
    return 2.0*PI*deltaTurn; // радианы
}
float MotorBlock::getTraveledDistance()
{   
    float ovTurn_k0 = encoder->overallTurnEnc_k0;
    float ovTurn_k1 = encoder->getOverallTurn();

    // Расчет пройденного расстояния колесом
    distanceTraveled_k1 = distanceTraveled_k0 + 2 * PI * wheelRadius * (ovTurn_k1 - ovTurn_k0) / 4095.0;
    // distanceTraveled_k1 = 2 * PI * R * (ovTurn_k1 - ovTurn_k0) / 4095.0;

    // Обновление значений
    distanceTraveled_k0 = distanceTraveled_k1; // по пройденному расстоянию
    //encoder->overallTurnEnc_k0 = ovTurn_k1;    // по абсолютному углу энкодера

    return distanceTraveled_k1;
}
// float MotorBlock::getTraveledDistance()
// {   
//     float ovTurn_k0 = encoder->overallTurnEnc_k0;
//     float ovTurn_k1 = encoder->getOverallTurn();
//     float R = getRadiusWheels();


//     // Расчет пройденного расстояния колесом
//     //distanceTraveled_k1 = distanceTraveled_k0 + 2 * PI * R * (ovTurn_k1 - ovTurn_k0) / 4095.0;
//     deltaDist = 2 * PI * R * (ovTurn_k1 - ovTurn_k0) / 4095.0;

//     // Обновление значений
//     //distanceTraveled_k0 = distanceTraveled_k1; // по пройденному расстоянию
//     encoder->overallTurnEnc_k0 = ovTurn_k1;    // по абсолютному улглу энкодера

//     return deltaDist;
// }
float MotorBlock::getRotAngle()
{
    float ovTurn_curr = encoder->getOverallTurn(); // число пройденных делений

    // return ovTurn_curr * 360.0 / 4095.0; // градусы
    // return ovTurn_curr * 2 * 3.141593 / 4095.0 // радианы
    return ovTurn_curr / 4095.0; // [об]
}
float MotorBlock::getAngleOffset()
{
    float ang = angleOffset;
    angleOffset = 0;
    return ang;
}
float MotorBlock::getLastSpeed() 
{
    return lastSpeed;
}