#include "motionControllerByPID.h"

MotionControllerByPID::MotionControllerByPID() {
    driveController = new DiffDriveControllerByPID();
}
MotionControllerByPID::MotionControllerByPID(float d, float rL, float rR, float v_max) {
    driveController = new DiffDriveControllerByPID(d, rL, rR, v_max);
}
MotionControllerByPID::~MotionControllerByPID() {
    delete driveController;
}

void MotionControllerByPID::setRobotConstant(float d, float rL, float rR, float v_max) {
    driveController->setRobotConstant(d, rL, rR, v_max);
}
void MotionControllerByPID::setCoefficient(float Kp, float Ki, float Kd) {
    driveController->setCoefficient(Kp, Ki, Kd);
}
void MotionControllerByPID::setDelay(float del) {
    this->del = del;
}

void MotionControllerByPID::setActualPosition(float x, float y, float alpha) {
    this->x = x;
    this->y = y;
    this->alpha = alpha;
}
void MotionControllerByPID::setTargetPosition(float _x, float _y) {
    this->_x = _x;
    this->_y = _y;
    setBetaIsUsed(false);
}
void MotionControllerByPID::setTargetPosition(float _x, float _y, float beta) {
    this->_x = _x;
    this->_y = _y;
    this->beta = beta;
    setBetaIsUsed(true);
}
void MotionControllerByPID::setBetaIsUsed(bool isUsedBeta) {
    this->isUsedBeta = isUsedBeta;
}

void MotionControllerByPID::updateVelocity() {
    // vec_act - текущий вектор направления робота
    float vec_act_x = cos(alpha);
    float vec_act_y = sin(alpha);

    // vec_crs - вектор между текущей и конечной точками
    float vec_delta_x = _x - x;
    float vec_delta_y = _y - y;

    delta = angleBetweenVectorsWithSign(vec_act_x, vec_act_y, vec_delta_x, vec_delta_y);
    distance = sqrt(vec_delta_x*vec_delta_x + vec_delta_y*vec_delta_y);
    
    //if (isUsedBeta)     // !!! Исправить позже
    driveController->updateVelocity(delta, distance, del);
    
    v = driveController->get_v();
    w = driveController->get_w();
    wheelL = driveController->get_wheelL();
    wheelR = driveController->get_wheelR();
}

float MotionControllerByPID::angleBetweenVectorsWithSign(float a_x, float a_y, float b_x, float b_y) {
    float a_hyp = sqrt(a_x*a_x + a_y*a_y);
    float b_hyp = sqrt(b_x*b_x + b_y*b_y);
    a_x /= a_hyp;
    a_y /= a_hyp;
    b_x /= b_hyp;
    b_y /= b_hyp;
    
    float angle = acos( (a_x*b_x + a_y*b_y) / (sqrt(a_x*a_x + a_y*a_y)*sqrt(b_x*b_x + b_y*b_y)) );       
    float alpha = acos( a_x / (sqrt(a_x*a_x + a_y*a_y)) );
    float beta = acos( b_x / (sqrt(b_x*b_x + b_y*b_y)) );
    if (alpha >= 0 and alpha <= M_PI/2) {
        if (a_y >= 0) {
            if (b_y >= 0 and (alpha > beta))
                angle *= -1;
            else if (b_y < 0 and (M_PI-alpha > beta))
                angle *= -1;
        } else {
            if (b_y < 0 and (alpha < beta))
                angle *= -1;
            else if (b_y >= 0 and (M_PI-alpha < beta))
                angle *= -1;
        }
    } else if (alpha > M_PI/2) {
        if (a_y >= 0) {
            if (b_y >= 0 and (alpha > beta))
                angle *= -1;
            else if (b_y < 0 and (M_PI-alpha > beta))
                angle *= -1;
        } else {
            if (b_y < 0 and (alpha < beta))
                angle *= -1;
            else if (b_y >= 0 and (M_PI-alpha < beta))
                angle *= -1;
        }
    }
    
    return angle;
}

bool MotionControllerByPID::isReachPosition() {
    float shift_x = x - _x;
    float shift_y = y - _y;
    return sqrt(shift_x*shift_x + shift_y*shift_y) < available_pos_err;
}

float MotionControllerByPID::get_delta() { return delta; }
float MotionControllerByPID::get_distance() { return distance; }
float MotionControllerByPID::get_v() { return v; }
float MotionControllerByPID::get_w() { return w; }
float MotionControllerByPID::get_wheelL() { return wheelL; }
float MotionControllerByPID::get_wheelR() { return wheelR; }