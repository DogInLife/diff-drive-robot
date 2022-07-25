#include "velocity.h"

Velocity::Velocity()
: ang(0), lin(0), maxRobot(0), maxWheel(0)
{}


float Velocity::computeLinearSpeed()
{
   if(fabs(ang) < 0.5)
      return maxRobot*4*square(0.5 - fabs(ang));
   else 
      return 0; 

   //return 0.6*maxRobot/(square(fabs(ang)) + 1); // =======  А ЭТУ ШТУКУ МОЖНО ИСПОЛЬЗОВАТЬ КАК-ТО ПРИ УГЛОВАТЫХ ТРАЕКТОРИЯХ
   //return maxRobot*2.0/5.0;
}

