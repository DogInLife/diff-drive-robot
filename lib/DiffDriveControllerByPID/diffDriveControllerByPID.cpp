#include "diffDriveControllerByPID.h"

DiffDriveControllerByPID::DiffDriveControllerByPID() {
    angularPID = new PID(Kp, Ki, Kd);
}
DiffDriveControllerByPID::DiffDriveControllerByPID(float d, float rL, float rR, float v_max) {
    setRobotConstant(d, rL, rR, v_max);
    angularPID = new PID(Kp, Ki, Kd);
}
DiffDriveControllerByPID::~DiffDriveControllerByPID() {
    delete angularPID;
}

void DiffDriveControllerByPID::setRobotConstant(float d, float rL, float rR, float v_max) {
    this->d = d;
    this->rL = rL;
    this->rR = rR;
    this->v_max = v_max;
}
void DiffDriveControllerByPID::setCoefficient(float Kp, float Ki, float Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    angularPID->setCoefficient(Kp, Ki, Kd);
    angularPID->resetErr();
}

void DiffDriveControllerByPID::updateVelocity(float delta, float distance, float del) {
    this->delta = -delta;
    this->distance = distance;          // добавить плавное ускорение

    if (abs(delta) > 0.52) {
        v = 0;
        w = angularPID->updateErr(delta, del);
    }
    else {
        v = v_max;
        if (distance < 0.2) {
            v = v * 5* distance;
            if (v < 0.1)
                v = 0.1;
        }
        w = angularPID->updateErr(delta, del);
    }
    calculateWheelSpeed();
}

void DiffDriveControllerByPID::calculateWheelSpeed() {
    wheelL = (v - w*d/2) / rL;
    wheelR = (v + w*d/2) / rR; 
}

float DiffDriveControllerByPID::get_v() { return v; }
float DiffDriveControllerByPID::get_w() { return w; }
float DiffDriveControllerByPID::get_wheelL() { return wheelL; }
float DiffDriveControllerByPID::get_wheelR() { return wheelR; }
float DiffDriveControllerByPID::get_error() { return angularPID->getErr(); }