#include "diffDriveController.h"

DiffDriveController::DiffDriveController() {}
DiffDriveController::DiffDriveController(float d, float rL, float rR, float v_max) {
    setRobotConstant(d, rL, rR, v_max);
}
DiffDriveController::~DiffDriveController() {}

void DiffDriveController::setRobotConstant(float d, float rL, float rR, float v_max) {
    this->d = d;
    this->rL = rL;
    this->rR = rR;
    this->v_max = v_max;
}
void DiffDriveController::setCoefficient(float k1, float k2, float k3, float k4, float K_max) {
    this->k1 = k1;
    this->k2 = k2;
    this->k3 = k3;
    this->k4 = k4;
    this->K_max = K_max;
}

void DiffDriveController::updateVelocity(float delta, float distance, float theta) {
    this->delta = delta;
    this->distance = distance;
    this->theta = theta;
    calculateTrajectoryCurvature();
    calculateVelocity();
    calculateWheelSpeed();
}

void DiffDriveController::calculateTrajectoryCurvature() {
    K = -(1/distance)*(k2*(delta-atan(-k1*theta))+(1+k1/(1+pow(k1*theta,2)))*sin(delta));
}
void DiffDriveController::calculateVelocity() {
    int sign_K = (K >= 0 ? 1 : -1); // знак K
    
    if (abs(K) >= K_max) {
        v = 0;
        w = k4 * v_max * sign_K;
    } else if (abs(k3*K) < 1) {
        v = v_max / (1+pow(k3*K,2));
        w = K * v_max / (1+pow(k3*K,2));
    } else {
        v = v_max / (2*k3*K) * sign_K;
        w = K * v * sign_K;
    }    
}
void DiffDriveController::calculateWheelSpeed() {
    wheelL = (v - w*d/2) / rL;
    wheelR = (v + w*d/2) / rR; 
}



float DiffDriveController::get_K() { return K; }
float DiffDriveController::get_v() { return v; }
float DiffDriveController::get_w() { return w; }
float DiffDriveController::get_wheelL() { return wheelL; }
float DiffDriveController::get_wheelR() { return wheelR; }