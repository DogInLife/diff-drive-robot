#include "motorBlock.h"

MotorBlock::MotorBlock(float wheelRadius, float maxVel, byte encPin, byte driverPin1, byte driverPin2, byte driverPinPWM) {
    setWheelRadius(wheelRadius);
    setMaxVel(maxVel);
    encoder = new Encoder();
    setEncorerPin(encPin);
    setDriverPin(driverPin1, driverPin2, driverPinPWM);
}

MotorBlock::~MotorBlock()
{
    delete encoder;
}

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

void MotorBlock::stopMoving()
{   
    for (int i = pwm; i != 0 ;)
    {
        i=i-10;
        i=(i<0)?0:i;
        analogWrite(PWM_PIN, i);
    }
}
void MotorBlock::updateVelocity(float vel)
{
    pwm = map(abs(vel), 0, maxVel, 0, 255);

    if (vel > 0)
    {
        digitalWrite(IN_DRIVER_PIN_1, LOW);
        digitalWrite(IN_DRIVER_PIN_2, HIGH);
    } 
    else 
    {
        digitalWrite(IN_DRIVER_PIN_1, HIGH);
        digitalWrite(IN_DRIVER_PIN_2, LOW);
    }
    analogWrite(PWM_PIN, pwm);
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
    //encoder->overallTurnEnc_k0 = ovTurn_k1;    // по абсолютному улглу энкодера

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

float MotorBlock::getDeltaAngle()
{
    float ovAng_prev = encoder->overallTurnEnc_k0; // число пройденных делений в предыдущий момент
    float ovAng_curr = encoder->getOverallTurn(); // число пройденных делений в текущий момент
    float deltaTurn = (ovAng_curr - ovAng_prev) / 4095.0; // переход от делений к оборотам

    encoder->overallTurnEnc_k0 = ovAng_curr;

    return 2.0*3.141593*deltaTurn; // радианы
}