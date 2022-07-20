#include "velocity.h"

Velocity::Velocity()
: ang(0), lin(0), maxRobot(0), maxWheel(0)
{}


float Velocity::computeLinearSpeed()
{
   //return maxRobot/(2*square(fabs(ang)) + 1); // =======  А ЭТУ ШТУКУ МОЖНО ИСПОЛЬЗОВАТЬ КАК-ТО ПРИ УГЛОВАТЫХ ТРАЕКТОРИЯХ
   return maxRobot*3.0/5.0;
}

