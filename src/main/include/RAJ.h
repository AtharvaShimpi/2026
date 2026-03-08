#pragma once
#include "math.h"
double RAJ(double distanceFromTarget, double kP);
double RAJ(double target, double currentPos, double kPDivisor);
double motorRAJ(double target, double currentPos, double Velocity, double decelerateDistance, double acceleration);
double clamp(double value, double min, double max);