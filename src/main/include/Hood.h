#include "Includes.h"

class Hood
{
    private:
        TalonFX &hoodMotor;
        double gearRatio;
        double kP;
        double kI;
        double kD;
        double integral;
        double prevError;

    public:
        Hood(TalonFX &hoodMotor, double gearRatio, double kI, double kP, double kD);
        double setHoodToAnglePID(double angle);
};
