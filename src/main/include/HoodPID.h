#pragma once
#include "Includes.h"

double hoodPrevError = 0;
double hoodDerivative = 0;
double hoodTotalError = 0;

double hoodPID(double targetPosition, double currentPosition, double kP, double kI, double kD, double activeZone)
{
  
    double error = targetPosition - currentPosition;
    hoodTotalError += error;
    if(fabs(error) > activeZone)
    {
        hoodTotalError = 0;
    }
    SmartDashboard::PutNumber("Hood Error",error);
    SmartDashboard::PutNumber("Hood Target Position", targetPosition);
    
    hoodDerivative = error - hoodPrevError;
    hoodPrevError = error;
    return  error*kP + hoodDerivative*kD;
}