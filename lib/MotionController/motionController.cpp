#include "motionController.h"

MotionController::MotionController() {
    driveController = new DiffDriveController(); 
}
MotionController::MotionController(float d, float rL, float rR, float v_max) {
    driveController = new DiffDriveController(d, rL, rR, v_max);
}
MotionController::~MotionController() {
    delete driveController;
}

void MotionController::setRobotConstant(float d, float rL, float rR, float v_max) {
    driveController->setRobotConstant(d, rL, rR, v_max);
}
void MotionController::setCoefficient(float k1, float k2, float k3, float k4, float K_max) {
    driveController->setCoefficient(k1, k2, k3, k4, K_max);
}

void MotionController::setActualPosition(float x, float y, float alpha) {
    this->x = x;
    this->y = y;
    this->alpha = alpha;
}
void MotionController::setTargetPosition(float _x, float _y) {
    this->_x = _x;
    this->_y = _y;
}
void MotionController::setTargetPosition(float _x, float _y, float beta) {
    this->_x = _x;
    this->_y = _y;
    this->beta = beta;
}
void MotionController::setBetaIsUsed(bool isUsedBeta) {
    this->isUsedBeta = isUsedBeta;
}

void MotionController::updateVelocity() {
    // vec_act - текущий вектор направления робота
    float vec_act_x = x*cos(alpha);
    float vec_act_y = y*cos(alpha);

    // vec_crs - вектор между текущей и конечной точками
    float vec_delta_x = _x - x;
    float vec_delta_y = _y - y;

    delta = angleBetweenVectorsWithSign(vec_act_x, vec_act_y, vec_delta_x, vec_delta_y);
    distance = sqrt(vec_delta_x*vec_delta_x + vec_delta_y*vec_delta_y);
    if (isUsedBeta)
        theta = 0;
    else
        theta = 0;      // !!! Исправить позже

    driveController->updateVelocity(delta, distance, theta);
    v = driveController->get_v();
    w = driveController->get_w();
    wheelL = driveController->get_wheelL();
    wheelR = driveController->get_wheelR();
}

float MotionController::angleBetweenVectorsWithSign(float a_x, float a_y, float b_x, float b_y) {
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
    
    return -angle;
}

bool MotionController::isReachPosition() {
    float shift_x = x - _x;
    float shift_y = y - _y;
    return (abs(sqrt(shift_x*shift_x + shift_y*shift_y)) / available_pos_err + 
            abs(theta - delta) / available_ang_err) / 2 < available_reach_err;
}

float MotionController::get_K() { return driveController->get_K(); }
float MotionController::get_delta() { return delta; }
float MotionController::get_distance() { return distance; }
float MotionController::get_theta() { return theta; }
float MotionController::get_v() { return v; }
float MotionController::get_w() { return w; }
float MotionController::get_wheelL() { return wheelL; }
float MotionController::get_wheelR() { return wheelR; }