#include "base/wheel.h"
#include "pid.h"

#ifndef _RobotState

double robotBaseWidth, robotWheelDiameter, distancePerPulseLeft, distancePerPulseRight;
double maxAngularVelocity, maxLinearVelocity;
double bound = 1;

bool pidDebug;

Pid *pidLeft, *pidRight;

base::wheel wheelTargetSpeed;
base::wheel wheelControls;

#define _RobotState
#endif