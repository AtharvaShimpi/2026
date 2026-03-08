#pragma once
#include "Includes.h"

class HDrive
{
private:
    TalonFX &leftMotor;
    TalonFX &rightMotor;
    TalonFX &hMotor;
    double mainRatio;
    double strafeRatio;
    double mainWheelDiameter;
    double strafeWheelDiameter;
    double previousLeftPos;
    double previousRightPos;
    double previousHPos;
public:
    studica::AHRS navx{studica::AHRS::NavXComType::kMXP_SPI};
    double X;
    double Y;
    HDrive(TalonFX &leftMotor, TalonFX &rightMotor, double MainRatio, double MainWheelDiameter, TalonFX &hMotor, double StrafeRatio, double StrafeWheelDiameter);
    void SetDrive(double leftSpeed, double rightSpeed, double strafeSpeed);
    void SetLeftMotor(double speed);
    void SetRightMotor(double speed);
    void SetHMotor(double speed);
    double GetLeftMotorSpeed();
    double GetRightMotorSpeed();
    double GetHMotorSpeed();
    double GetLeftMotorPosition();
    double GetRightMotorPosition();
    double GetHMotorPosition();
    void SetPosition(double x, double y);
    void UpdatePosition();
    void SetDriveHolonomic(double horizontal, double vertical, double rotation);
    void SetDriveFancyHolonomic(double horizontal, double vertical, double targetAngle, double adjustangle = 100);
};
