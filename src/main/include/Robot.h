#define DEFAULT_PRESSURE 35
#define DEFAULT_IS_HOLONOMIC false
#define POWER_DISTRIBUTION_HUB 62, frc::PneumaticsModuleType::REVPH
#define HDRIVE_PISTON 10
#define INTAKE_PISTON 15

#define RIGHT_MOTOR 1
#define LEFT_MOTOR 2
#define H_MOTOR 3
#define FLYWHEEL_MOTOR 5


#define DRIVE_RATIO 12
#define DRIVE_WHEEL_DIAMETER 6
#define HDRIVE_RATIO 12
#define HDRIVE_WHEEL_DIAMETER 4
#define DRIVE_DIAMETER 25.25

#define CREATE_PERSISTENT_NUMBER(name,value)  SmartDashboard::PutNumber(name, SmartDashboard::GetNumber(name, value)); SmartDashboard::SetPersistent(name)
#define CREATE_PERSISTENT_BOOLEAN(name,value) SmartDashboard::PutBoolean(name, SmartDashboard::GetBoolean(name, value)); SmartDashboard::SetPersistent(name)

#pragma once
#include "Includes.h"   
#include "Controller.h"
#include "HDrive.h"
class Robot : public frc::TimedRobot {
 public:
  Compressor compressor{POWER_DISTRIBUTION_HUB};
  Solenoid hdrive_piston{POWER_DISTRIBUTION_HUB, HDRIVE_PISTON};
  Solenoid intake_piston{POWER_DISTRIBUTION_HUB, INTAKE_PISTON};
  TalonFX rightMotor{RIGHT_MOTOR};
  TalonFX leftMotor{LEFT_MOTOR};
  
  TalonFX flywheelMotor{FLYWHEEL_MOTOR};
  TalonFX flywheelMotor2{7};
  TalonFX indexerMotor{8};

  SparkMax hoodMotor = SparkMax(6,rev::spark::SparkLowLevel::MotorType::kBrushless);
  
 TalonFX intakeMotor{4};
  TalonFX hMotor{H_MOTOR};
    inline static const frc::AprilTagFieldLayout kTagLayout{frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)};
    inline static const frc::Transform3d RelativeFrontCameraPosition{frc::Translation3d{0.5_m, 0.0_m, 0.5_m},frc::Rotation3d{0_rad, 0_rad, 0_rad}};
    photon::PhotonPoseEstimator photonEstimator{kTagLayout,RelativeFrontCameraPosition};  
    photon::PhotonCamera camera{"Front Camera"};
    std::optional<photon::EstimatedRobotPose> visionEst;
    frc::Pose3d robotPose3d;
  double weldedTagPoses[33][4] = {
    {0.00,0.00,0.00,0},
    {467.64, 292.31, 35.00, 180}, // 1
    {469.11, 182.60, 44.25, 90},  // 2
    {445.35, 172.84, 44.25, 180}, // 3
    {445.35, 158.84, 44.25, 180}, // 4
    {469.11, 135.09, 44.25, 270}, // 5
    {467.64, 25.37, 35.00, 180},  // 6
    {470.59, 25.37, 35.00, 0},    // 7
    {483.11, 135.09, 44.25, 270}, // 8
    {492.88, 144.84, 44.25, 0},   // 9
    {492.88, 158.84, 44.25, 0},   // 10
    {483.11, 182.60, 44.25, 90},  // 11
    {470.59, 292.31, 35.00, 0},   // 12
    {650.92, 291.47, 21.75, 180}, // 13
    {650.92, 274.47, 21.75, 180}, // 14
    {650.90, 170.22, 21.75, 180}, // 15
    {650.90, 153.22, 21.75, 180}, // 16
    {183.59, 25.37, 35.00, 0},    // 17
    {182.11, 135.09, 44.25, 270}, // 18
    {205.87, 144.84, 44.25, 0},   // 19
    {205.87, 158.84, 44.25, 0},   // 20
    {182.11, 182.60, 44.25, 90},  // 21
    {183.59, 292.31, 35.00, 0},   // 22
    {180.64, 292.31, 35.00, 180}, // 23
    {168.11, 182.60, 44.25, 90},  // 24
    {158.34, 172.84, 44.25, 180}, // 25
    {158.34, 158.84, 44.25, 180}, // 26
    {168.11, 135.09, 44.25, 270}, // 27
    {180.64, 25.37, 35.00, 180},  // 28
    {0.30, 26.22, 21.75, 0},      // 29
    {0.30, 43.22, 21.75, 0},      // 30
    {0.32, 147.47, 21.75, 0},     // 31
    {0.32, 164.47, 21.75, 0}      // 32
  };
  double andymarkTagPoses[33][4] = {
    {0.00,0.00,0.00,0},
    {467.08, 291.79, 35.00, 180}, // 1
    {468.56, 182.08, 44.25, 90},  // 2
    {444.80, 172.32, 44.25, 180}, // 3
    {444.80, 158.32, 44.25, 180}, // 4
    {468.56, 134.56, 44.25, 270}, // 5
    {467.08, 24.85, 35.00, 180},  // 6
    {470.03, 24.85, 35.00, 0},    // 7
    {482.56, 134.56, 44.25, 270}, // 8
    {492.33, 144.32, 44.25, 0},   // 9
    {492.33, 158.32, 44.25, 0},   // 10
    {482.56, 182.08, 44.25, 90},  // 11
    {470.03, 291.79, 35.00, 0},   // 12
    {649.58, 291.02, 21.75, 180}, // 13
    {649.58, 274.02, 21.75, 180}, // 14
    {649.57, 169.78, 21.75, 180}, // 15
    {649.57, 152.78, 21.75, 180}, // 16
    {183.03, 24.85, 35.00, 0},    // 17
    {181.56, 134.56, 44.25, 270}, // 18
    {205.32, 144.32, 44.25, 0},   // 19
    {205.32, 158.32, 44.25, 0},   // 20
    {181.56, 182.08, 44.25, 90},  // 21
    {183.03, 291.79, 35.00, 0},   // 22
    {180.08, 291.79, 35.00, 180}, // 23
    {167.56, 182.08, 44.25, 90},  // 24
    {157.79, 172.32, 44.25, 180}, // 25
    {157.79, 158.32, 44.25, 180}, // 26
    {167.56, 134.56, 44.25, 270}, // 27
    {180.08, 24.85, 35.00, 180},  // 28
    {0.54, 25.62, 21.75, 0},      // 29
    {0.54, 42.62, 21.75, 0},      // 30
    {0.55, 146.86, 21.75, 0},     // 31
    {0.55, 163.86, 21.75, 0}      // 32
  };

  //TalonFX flywheel{4};+
  HDrive drive{leftMotor, rightMotor, DRIVE_RATIO, DRIVE_WHEEL_DIAMETER, hMotor, HDRIVE_RATIO, HDRIVE_WHEEL_DIAMETER};
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;
  void AprilTagThread();

 private:
  Controller controller{0};
};
