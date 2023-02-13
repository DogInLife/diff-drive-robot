#ifndef TWO_WHEELED_ROBOT_H
#define TWO_WHEELED_ROBOT_H

#include "motorBlock.h"
#include "pid.h"
#include "velocity.h"
#include "position.h"
#include "math.h"

#include "GracefulMotionControler.h"
#include "RFIDReader.h"
//#include <MFRC522.h>
#include "hallSensor.h"
#include "magneticLineSensor.h"

class TwoWheeledRobot
{
private:
    //MFRC522* rfidReader;
    RFIDReader* rfidReader;

    MagneticLineReader* magneticLineReader;

    MotorBlock* motorBlockL;
    MotorBlock* motorBlockR;
    PID* pidL;
    PID* pidR;
    PID* pid;
    Velocity vel;
    Position pos;

    MotionControler* motionControler;

    float baseLength;
    byte PIN_CURRENT_SENSOR;

    bool reachedGoal;
    bool globalStop;
    int newMinRange; // Для функции map в setVelocity
    byte inByte;

public:
    TwoWheeledRobot();
    ~TwoWheeledRobot();

    void createWheels(float wheelRadius, float baseLength, float maxVel);
    //void createRFIDReader();

    // SET
    void setEncoderPins(byte encPinL, byte encPinR);
    void setDriverPins(byte driverPinPWM_R, byte driverPin_R2, byte driverPin_R1, byte driverPin_L1, byte driverPin_L2, byte driverPinPWM_L);
    // GET
    float getRadiusWheels();
    byte getSerialData();
    
    void tuneWhlPID(float KpL, float KiL, float KdL, float KpR, float KiR, float KdR);
    void tunePID(float Kp, float Ki, float Kd);
    
// ========= control ===========
    void serialControl(int del, bool deb);
    void globalSerialControl();
    void globalSerialControl(byte inB);
    
// ========= behavior ===========
    void manualControl(int dt);
    void resertPosition();
    void goToPosition(float x, float y, int del, bool deb);
    void turnAngle(float theta, int del, bool deb);
    void goToNULL(int del, bool deb);
    void goTrack(Position points[], int del, bool deb);
    void goCWtest(float L, int del, bool deb);
    void goCCWtest(float L, int del, bool deb);
    void goCircle(float radius, int ptsNum, bool deb, int circles);
    int goToGoal(float x_d, float y_d, bool isFinish, int del, bool deb, bool followRFID, int idRFID);

    void goMagneticLine(int del, bool deb);
    void goMagneticLine2Point(float _x, float _y, int del, bool deb);
    int moveDecision(float _x, float _y, int crossroadsCounter);
    
    void rfidTest(int del);
    void rot_test(int whl_vel_des, byte del, bool deb, float xGoal, float yGoal); // ####################################

    void goForward(int velL, int velR);
    void turnLeft(int velL, int velR);
    void turnRight(int velL, int velR);
    void stopMoving();
    int checkCurrent(byte PIN_CURRENT_SENSOR);
    // m/s to об/min
    int linear2angular(int vel);                
};

#endif // TWO_WHEELED_ROBOT_H
