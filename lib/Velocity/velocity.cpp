#include "velocity.h"

Velocity::Velocity()
: ang(0), lin(0), maxRobot(0), maxWheel(0)
{}


float Velocity::computeLinearSpeed()
{
   return 0.4*maxRobot/(square(fabs(ang) + 1)); // =======  А ЭТУ ШТУКУ МОЖНО ИСПОЛЬЗОВАТЬ КАК-ТО ПРИ УГЛОВАТЫХ ТРАЕКТОРИЯХ
   //return maxRobot*2.0/5.0;
}

