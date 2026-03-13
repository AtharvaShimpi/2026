#include "Robot.h"
#include "FlywheelPID.h"
#include "HoodPID.h"

// nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
// std::shared_ptr<nt::NetworkTable> visiontable = inst.GetTable("photonvision");
// std::shared_ptr<nt::NetworkTable> cameratable = visiontable->GetSubTable("HD_Pro_Webcam_C920 (2)");
// photon::PhotonCamera camera{"Front_Camera"};
// photon::PhotonPipelineResult result = camera.GetLatestResult();
// std::shared_ptr<nt::NetworkTable> targetPose = cameratable->GetSubTable("taret-pose");
// double prevPosition = 0;g
// bool hasTarget = result.HasTargets();
// photon::PhotonTrackedTarget target = result.GetBestTarget();
double distance1 = 0;
bool isclose = false;
frc::Pose3d targetPose = LimelightHelpers::getTargetPose3d_CameraSpace("");
Robot::Robot()
{
}
void Robot::AprilTagThread()
{
  // auto visionEst = photonEstimator.EstimateCoprocMultiTagPose(result);
  if (!visionEst)
  {
    // visionEst = photonEstimator.EstimateLowestAmbiguityPose(result);
  }
  if (visionEst)
  {
    robotPose3d = visionEst->estimatedPose;
    // drive.SetPosition(robotPose3d.X().value(),robotPose3d.Y().value())
  }
}

void Robot::RobotInit()
{
  SmartDashboard::PutBoolean("isHolonomic", SmartDashboard::GetNumber("isHolonomic", DEFAULT_IS_HOLONOMIC));
  SmartDashboard::PutBoolean("Flywheel On", true);
  CREATE_PERSISTENT_NUMBER("Pressure", DEFAULT_PRESSURE);
  CREATE_PERSISTENT_NUMBER("Hood kP", 0.6);
  CREATE_PERSISTENT_NUMBER("Hood kI", 0.0);
  CREATE_PERSISTENT_NUMBER("Hood kD", 0.0);
  CREATE_PERSISTENT_NUMBER("Hood Active Zone", 0.0);
  CREATE_PERSISTENT_NUMBER("Hood Angle", 1);
  CREATE_PERSISTENT_NUMBER("Intake Power", 1);
  CREATE_PERSISTENT_NUMBER("kP", 1.04);
  CREATE_PERSISTENT_NUMBER("kI", 0.0098);
  CREATE_PERSISTENT_NUMBER("kD", 0.77);
  CREATE_PERSISTENT_NUMBER("Active Zone", 0.8);
  CREATE_PERSISTENT_NUMBER("Flywheel Target", 75);
  CREATE_PERSISTENT_NUMBER("HoodProportion", 1);
  CREATE_PERSISTENT_NUMBER("close", 80);
  CREATE_PERSISTENT_NUMBER("far", 80);
  CREATE_PERSISTENT_NUMBER("Hood Target", 60);
  CREATE_PERSISTENT_NUMBER("Down Limit", 0.5);
  CREATE_PERSISTENT_NUMBER("Auton Selection", 0);
  std::thread vThread([this]
                      { AprilTagThread(); });
  vThread.detach();
}

void Robot::RobotPeriodic()
{ 
  SmartDashboard::PutNumber("Match Time", HAL_GetMatchTime(nullptr));
  SmartDashboard::PutNumber("x", drive.X);
  SmartDashboard::PutNumber("y", drive.Y);
  SmartDashboard::PutNumber("Hood Position", hoodMotor.GetEncoder().GetPosition() * 360.0 / 43.75);
  SmartDashboard::PutNumber("LeftSpeed", drive.GetLeftMotorSpeed());
  SmartDashboard::PutNumber("RightSpeed", drive.GetRightMotorSpeed());
  SmartDashboard::PutNumber("HSpeed", drive.GetHMotorSpeed());
  SmartDashboard::PutNumber("Current Pressure", compressor.GetPressure().value());
  SmartDashboard::PutNumber("Voltage", RobotController::GetBatteryVoltage().value());
  SmartDashboard::PutNumber("Angle", drive.navx.GetYaw());
  SmartDashboard::PutNumber("Flywheel Speed", flywheelMotor.GetVelocity().GetValueAsDouble());
  SmartDashboard::PutNumber("Distance", distance1);
  // frc::AprilTagFieldLayout field = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
  // cameratable->GetEntry("targetYaw").GetDouble(0);
  //  frc2::CommandScheduler::GetInstance().Run();

  for (const auto &result : camera.GetAllUnreadResults())
  {
    if (result.HasTargets())
    {
      auto target = result.GetBestTarget();

      auto camToTarget = target.GetBestCameraToTarget();

      double forward = camToTarget.X().to<double>();
      double sideways = camToTarget.Y().to<double>();
      SmartDashboard::PutBoolean("DistanceSeen", true);
      distance1 = std::sqrt(forward * forward + sideways * sideways);
    }
  }
  drive.UpdatePosition();
  // double magnitude = robotPose3d.X().to<double>();
  // SmartDashboard::PutNumber("DistanceFromAprilTag", magnitude);
}

void Robot::DisabledInit()
{
}

void Robot::DisabledPeriodic()
{
}

void Robot::DisabledExit() {}
bool isCalibrated = false;
;
int autonvalue;
void Robot::AutonomousInit()
{
  compressor.EnableAnalog(units::pressure::pounds_per_square_inch_t(SmartDashboard::GetNumber("Pressure", DEFAULT_PRESSURE)), units::pressure::pounds_per_square_inch_t(SmartDashboard::GetNumber("Pressure", DEFAULT_PRESSURE) + 0.5));
  intake_piston.Set(true);
  indexerMotor.Set(-0.1);
  isCalibrated = false;
  autonvalue = SmartDashboard::GetNumber("Auton Selection",0);
}

void Robot::AutonomousPeriodic()
{
  switch(autonvalue)
  {
    case 0:

    case 1:
      double flywheelSpeed = flywheelPID(SmartDashboard::GetNumber("Flywheel Target", 75), flywheelMotor2.GetVelocity().GetValueAsDouble(), SmartDashboard::GetNumber("kP", 0.05), SmartDashboard::GetNumber("kI", 0.005), SmartDashboard::GetNumber("kD", 0.003), SmartDashboard::GetNumber("Active Zone", 300));
      flywheelMotor.Set(flywheelSpeed);
      flywheelMotor2.Set(flywheelSpeed);
      hoodMotor.Set(hoodPID(SmartDashboard::GetNumber("far", 60),hoodMotor.GetEncoder().GetPosition() * (360.0) / 43.75, SmartDashboard::GetNumber("Hood kP", 1.04), SmartDashboard::GetNumber("Hood kI", 0.0098), SmartDashboard::GetNumber("Hood kD", 0.77), SmartDashboard::GetNumber("Hood Active Zone", 0.8)) / 57.0);
      if (leftMotor.GetVelocity().GetValueAsDouble() > 0 && rightMotor.GetVelocity().GetValueAsDouble() > 0)
      {
        leftMotor.Set(0.3);
        rightMotor.Set(0.3);
      }
      else
      {
        leftMotor.Set(0.0);
        rightMotor.Set(0.0);
        indexerMotor.Set(1.0);
      }
  }

}

void Robot::AutonomousExit() {}

void Robot::TeleopInit()
{

  hdrive_piston.Set(false);
}
void Robot::TeleopPeriodic()
{

  //
  double xDistance = targetPose.X().value();
  compressor.EnableAnalog(units::pressure::pounds_per_square_inch_t(SmartDashboard::GetNumber("Pressure", DEFAULT_PRESSURE)), units::pressure::pounds_per_square_inch_t(SmartDashboard::GetNumber("Pressure", DEFAULT_PRESSURE) + 0.5));
  // double flywheelSpeed = flywheelPID(75,flywheelMotor2.GetVelocity().GetValueAsDouble(),SmartDashboard::GetNumber("kp",0.05),SmartDashboard::GetNumber("kI",0.005),SmartDashboard::GetNumber("kD",0.003),SmartDashboard::GetNumber("Active Zone",300))/100.0;
  double flywheelSpeed = flywheelPID(SmartDashboard::GetNumber("Flywheel Target", 75), flywheelMotor2.GetVelocity().GetValueAsDouble(), SmartDashboard::GetNumber("kP", 0.05), SmartDashboard::GetNumber("kI", 0.005), SmartDashboard::GetNumber("kD", 0.003), SmartDashboard::GetNumber("Active Zone", 300));
  SmartDashboard::PutNumber("Flywheel Set Speed", flywheelSpeed);
  if (SmartDashboard::GetBoolean("Flywheel On", true))
  {
    flywheelMotor.Set(flywheelSpeed);
    flywheelMotor2.Set(flywheelSpeed);
  }
  else
  {
    flywheelMotor2.StopMotor();
    flywheelMotor.StopMotor();
  }
  double target = distance1 * 10;
  if (target > 55)
  {
    target = 55;
  }
  double PID = hoodPID(SmartDashboard::GetNumber("Hood Target", 80) * SmartDashboard::GetNumber("HoodProportion", 0), hoodMotor.GetEncoder().GetPosition() * (360.0) / 43.75, SmartDashboard::GetNumber("Hood kP", 1.04), SmartDashboard::GetNumber("Hood kI", 0.0098), SmartDashboard::GetNumber("Hood kD", 0.77), SmartDashboard::GetNumber("Hood Active Zone", 0.8)) / 57.0;

  SmartDashboard::PutNumber("Hood PID", PID);
  if (PID > SmartDashboard::GetNumber("Down Limit", 0.5))
  {
    PID = SmartDashboard::GetNumber("Down Limit", 0.5);
  }
  if (isCalibrated == false)
  {
    hoodMotor.Set(0.1);
    if (hoodMotor.GetEncoder().GetVelocity() <= 0)
    {
      hoodMotor.GetEncoder().SetPosition((91.2 * 43.75) / 360.0);
      isCalibrated = true;
    }
  }
  else
  {
    hoodMotor.Set(PID);
  }
  if (controller.ButtonY.hasBeenBumped())
  {
    isclose = !isclose;
  }
  // hoodMotor.Set(hoodPID(SmartDashboard::GetNumber("hood Angle", 30),hoodMotor.GetEncoder().GetPosition(),1,0,0,10));,

  // SmartDashboard::PutNumber("Flywheel Speed Based On Position",flywheelMotor.GetPosition().GetValueAsDouble() - prevPosition);
  // prevPosition = flywheelMotor.GetPosition().GetValueAsDouble();

  // SmartDashboard::PutNumber("flywheel pid", flywheelPID(1,flywheel.GetVelocity().GetValueAsDouble()/512.0,0.01,0.0000001, 0.02, 0.05,0));
  // SmartDashboard::PutNumber("Flywheel velo", flywheel.GetVelocity().GetValueAsDouble() * 60);
  double Axis1 = controller.Axis1.position(); // right-left right
  double Axis2 = controller.Axis2.position(); // up-down right
  double Axis3 = controller.Axis3.position(); // up-down left
  double Axis4 = controller.Axis4.position(); // right-left left
  double HDrive = std::clamp(Axis1 + Axis4, -1.0, 1.0);

  // drive.SetDriveFancyHolonomic(Axis4,Axis3, cameratable->GetEntry("targetYaw").GetDouble(0),20);
  if (controller.ButtonR1.isPressing())
  {
    intakeMotor.Set(SmartDashboard::GetNumber("Intake Power", 1));
  }
  else if (controller.ButtonR2.isPressing())
  {
    intakeMotor.Set(-SmartDashboard::GetNumber("Intake Power", 1));
  }
  else
  {
    intakeMotor.Set(0);
  }
  // if(controller.ButtonUp.isPressing())
  // {
  //   hoodMotor.Set(SmartDashboard::GetNumber("Hood Speed",0));
  // }
  // else if(controller.ButtonDown.isPressing())
  // {
  //   hoodMotor.Set(-SmartDashboard::GetNumber("Hood Speed",0));
  // }
  // else
  // {
  //   hoodMotor.Set(0);
  // })
  if (controller.ButtonL1.isPressing())
  {
    indexerMotor.Set(1);
    intakeMotor.Set(-SmartDashboard::GetNumber("Intake Power", 1));
  }
  else
  {

    indexerMotor.StopMotor();
  }
  if (controller.ButtonDown.hasBeenBumped())
  {
    SmartDashboard::PutBoolean("isHolonomic", !SmartDashboard::GetBoolean("isHolonomic", DEFAULT_IS_HOLONOMIC));
  }

  if (SmartDashboard::GetBoolean("isHolonomic", DEFAULT_IS_HOLONOMIC))
  {
    drive.SetDriveHolonomic(Axis4, Axis3, Axis1);
  }
  else
  {
    drive.SetDrive(Axis3, Axis2, HDrive);
  }

  if (controller.ButtonA.isPressing())
  {
    drive.navx.ZeroYaw();
  }
  if (controller.ButtonX.isPressing())
  {
    drive.SetPosition(0, 0);
  }

  if (isclose)
  {
    SmartDashboard::PutNumber("Hood Target", SmartDashboard::GetNumber("close", 60));
  }
  else
  {
    SmartDashboard::PutNumber("Hood Target", SmartDashboard::GetNumber("far", 80));
  }

  // Button Bumped for pneumatics

  if (controller.ButtonRight.hasBeenBumped())
  {
    hdrive_piston.Toggle();
  }
  if (controller.ButtonLeft.hasBeenBumped())
  {
    intake_piston.Toggle();
  }

  // if(controller.ButtonR1.hasBeenBumped())
  // {
  //   piston1.Set(frc::DoubleSolenoid::Value::kReverse);
  //   piston2.Set(frc::DoubleSolenoid::Value::kReverse);
  // }
  // else
  // {
  //   piston1.Set(frc::DoubleSolenoid::Value::kForward);
  //   piston2.Set(frc::DoubleSolenoid::Value::kForward);
  // }
  // piston1.Set(frc::DoubleSolenoid::Value::kForward);
  // piston2.Set(frc::DoubleSolenoid::Value::kForward);
  // SmartDashboard::PutNumber("rpm",flywheelPID(0.3,flywheel.GetVelocity().GetValueAsDouble()/512.0,0.01,0.000001,0.02,0.05,0));
  controller.update();
}

void Robot::TeleopExit() {}

void Robot::TestInit()
{
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

int main()
{
  return frc::StartRobot<Robot>();
}
