#include "velocity.h"

Velocity::Velocity(float maxWheelRpm, float wheelR) {
   this->maxWheelRpm = maxWheelRpm;
   maxWheel = rmp2rads(maxWheelRpm);
   maxVelocity = angular2linear(maxWheel, wheelR);
}
Velocity::~Velocity() {}

void Velocity::setMaxWheelRpm(float maxWheelRpm, float wheelR) {
   this->maxWheelRpm = maxWheelRpm;
   maxWheel = rmp2rads(maxWheelRpm);
   maxVelocity = angular2linear(maxWheel, wheelR);
}

float Velocity::rmp2rads(float rmp) { return rmp * M_PI / 30; }   // 1 об/мин = π/30 рад/с
float Velocity::rads2rmp(float rads) { return rads * 30 / M_PI; }
float Velocity::angular2linear(float ang, float wheelR) { return ang * wheelR; }
float Velocity::linear2angular(float vel, float wheelR) { return vel / wheelR; }

float Velocity::getMaxWheelRpm() { return maxWheelRpm; }
float Velocity::getMaxWheel() { return maxWheel; }
float Velocity::getMaxVelocity() { return maxVelocity; }