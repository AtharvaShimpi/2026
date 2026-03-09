#include "HDrive.h"


HDrive::HDrive(TalonFX &leftMotor, TalonFX &rightMotor, double MainRatio, double MainWheelDiameter, TalonFX &hMotor, double StrafeRatio, double StrafeWheelDiameter)
    : leftMotor(leftMotor), rightMotor(rightMotor), mainRatio(MainRatio), hMotor(hMotor), strafeRatio(StrafeRatio)
{
    mainRatio = MainRatio;
    mainWheelDiameter = MainWheelDiameter;
    strafeRatio = StrafeRatio;
    strafeWheelDiameter = StrafeWheelDiameter;
    previousLeftPos = (leftMotor.GetPosition().GetValueAsDouble() / mainRatio) * mainWheelDiameter * M_PI;
    previousRightPos = (rightMotor.GetPosition().GetValueAsDouble() / mainRatio) * mainWheelDiameter * M_PI;
    previousHPos = (hMotor.GetPosition().GetValueAsDouble() / strafeRatio) * strafeWheelDiameter * M_PI;
}

void HDrive::SetDrive(double leftSpeed, double rightSpeed, double strafeSpeed)
{
    SetLeftMotor(leftSpeed);
    SetRightMotor(rightSpeed);
    SetHMotor(strafeSpeed);
}

void HDrive::SetDriveHolonomic(double horizontal, double vertical, double rotate)
{
    double distance = clamp(hypot(horizontal,vertical),-1.0,1.0);
    double angle = atan2(vertical,horizontal);
    angle = angle + navx.GetYaw()*M_PI/180;
    SetDrive(clamp(distance*sin(angle)+rotate,-1.0,1.0)*strafeRatio/mainRatio,clamp(distance*sin(angle)-rotate,-1.0,1.0)*strafeRatio/mainRatio, distance*cos(angle));
}

void HDrive::SetDriveFancyHolonomic(double horizontal, double vertical, double targetAngle, double adjustAngle)
{
    double magnitude = clamp(hypot(horizontal, vertical), -1.0, 1.0); //gets speed magnitude and converts it to a scalar with dimension 1

    double angle = atan2(vertical, horizontal); //gets angle from vertical and horizontal shifts

    angle = angle + navx.GetYaw() * 3.14159 / 180.0; //updates angle based on navX input

    double straightSpeed = magnitude*sin(angle);
    double changeInAngle = (targetAngle-angle) * adjustAngle;
    double horizontalSpeed = magnitude*cos(angle);
    SetDrive(
    SmartDashboard::PutNumber("Right speed", clamp(straightSpeed + changeInAngle, -1.0, 1.0) *
    strafeRatio / mainRatio),
    //Gets speed and converts it to vector, 
    //then converting the side drives speed to the HPod's speed to make it the same

    SmartDashboard::PutNumber("Left speed", clamp(straightSpeed - changeInAngle, -1.0, 1.0) *
    strafeRatio / mainRatio),
    //Gets speed and converts it to vector, 
    //then converting the side drives speed to the HPod's speed to make it the same

    SmartDashboard::PutNumber("H speed", horizontalSpeed)
    //Gets the Absolute Vector from the angle and distance
    );
}


void HDrive::SetLeftMotor(double speed)
{
    leftMotor.Set(speed);
}

void HDrive::SetRightMotor(double speed)
{
    rightMotor.Set(speed);
}

void HDrive::SetHMotor(double speed)
{
    hMotor.Set(speed);
}

double HDrive::GetLeftMotorSpeed()
{
    return leftMotor.GetRotorVelocity().GetValueAsDouble();
}

double HDrive::GetRightMotorSpeed()
{
    return rightMotor.GetRotorVelocity().GetValueAsDouble();
}

double HDrive::GetHMotorSpeed()
{
    return hMotor.GetRotorVelocity().GetValueAsDouble();
}

double HDrive::GetLeftMotorPosition()
{
    return leftMotor.GetPosition().GetValueAsDouble();
}

double HDrive::GetRightMotorPosition()
{
    return rightMotor.GetPosition().GetValueAsDouble();
}

double HDrive::GetHMotorPosition()
{
    return hMotor.GetPosition().GetValueAsDouble();
}
void HDrive::SetPosition(double x, double y)
{
    X=x;
    Y=y;
}
void HDrive::UpdatePosition()
{
    double LeftPos = (leftMotor.GetPosition().GetValueAsDouble() / mainRatio) * mainWheelDiameter * M_PI * 360;
    double RightPos = (rightMotor.GetPosition().GetValueAsDouble() / mainRatio) * mainWheelDiameter * M_PI * 360;
    double HPos = (hMotor.GetPosition().GetValueAsDouble() / strafeRatio) * strafeWheelDiameter * M_PI* 360;
    double averageSides = ((RightPos-previousRightPos+LeftPos-previousLeftPos)/2.0);
    double hDrive = HPos-previousHPos;
    X += (averageSides*sin(navx.GetYaw()*M_PI/180)+hDrive*cos(navx.GetYaw()*M_PI/180))*.001;
    Y += (averageSides*cos(navx.GetYaw()*M_PI/180)+hDrive*sin(navx.GetYaw()*M_PI/180))*.001;
    previousLeftPos = LeftPos;
    previousRightPos = RightPos;
    previousHPos = HPos;
}