#include "velocity.h"
#include "math.h"

Velocity::Velocity()
: ang(0), lin(0), maxRobot(0), maxWheel(0)
{}


float Velocity::computeLinearSpeed(float err)
{
   if(fabs(err) < 0.25)
      return 0.5*maxRobot*(0.3 - fabs(err))/0.25;
   else 
      return 0;

   //return 0.8*(0.5*maxRobot*(1 + cos(err)));
   //return 0.6*maxRobot/(square(fabs(ang)) + 1); // =======  А ЭТУ ШТУКУ МОЖНО ИСПОЛЬЗОВАТЬ КАК-ТО ПРИ УГЛОВАТЫХ ТРАЕКТОРИЯХ
   //return maxRobot*2.0/5.0;
}

