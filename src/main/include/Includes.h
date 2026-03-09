#pragma once
#define _USE_MATH_DEFINES
#include <frc2/command/button/CommandGenericHID.h>

#include <frc2/command/button/CommandJoystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/GenericHID.h>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/SerialPort.h>
#include <wpi/print.h>
#include <string>
#include <cmath>
#include <frc/Timer.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <algorithm>
#include <networktables/NetworkTableInstance.h>
#include "frc2/command/SequentialCommandGroup.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/CommandPtr.h"
#include "rev/SparkMax.h"
#include "elasticlib.h"
#include "studica/AHRS.h"
#include "cameraserver/CameraServer.h"
#include <networktables/NTSendableBuilder.h>
#include <networktables/NTSendable.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "frc/AnalogPotentiometer.h"
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include <thread>
#include "photon/PhotonCamera.h"
#include "photon/PhotonPoseEstimator.h"
#include <optional>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include "LimelightHelpers.h"

using namespace rev;
using namespace frc;
using namespace spark;
using namespace frc2;
using namespace std;
using namespace ctre::phoenix6::hardware;
