#include "velocity.h"

Velocity::Velocity()
: ang(0), lin(0), maxRobot(0), maxWheel(0)
{}


float Velocity::computeLinearSpeed()
{
   return maxRobot/(2*square(fabs(ang)) + 1);
}

