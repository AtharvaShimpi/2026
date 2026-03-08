#include "Hood.h"

Hood::Hood(TalonFX &hoodMotor,double gearRatio, double kI, double kP, double kD) :
 hoodMotor(hoodMotor), gearRatio(gearRatio), kP(kP), kI(kI), kD(kD)
{
    gearRatio = gearRatio;
    kP = kP;
    kI = kI;
    kD = kD;
};


double Hood::setHoodToAnglePID(double angle)
{
    return 1.0;
}
