#pragma once
#include "Includes.h"

double prevError = 0;
double derivative = 0;
double totalError = 0;

double flywheelPID(double targetSpeed, double currentSpeed, double kP, double kI, double kD, double activeZone)
{
    double error = targetSpeed - currentSpeed;
    totalError += error;
    if(fabs(error) > activeZone)
    {
        totalError = 0;
    }

    derivative = error - prevError;
    prevError = error;
    return (targetSpeed + (error*kP + totalError*kI + derivative*kD))/100.0;
}