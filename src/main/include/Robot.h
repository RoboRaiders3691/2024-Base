// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc2/command/Command.h>
#include <frc/motorcontrol/Talon.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/FieldObject2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/Odometry.h>
#include <frc/kinematics/Kinematics.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/ADXRS450_Gyro.h>
#include <units/angle.h>
#include <units/length.h>
#include <photon/PhotonCamera.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/estimator/MecanumDrivePoseEstimator.h>
#include <frc/Encoder.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <wpi/SymbolExports.h>
#define WPILIB_DLLEXPORT
class WPILIB_DLLEXPORT ObjectToRobotPose;
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/SpanExtras.h"
#include "networktables/NTSendableBuilder.h"
#include "networktables/NTSendable.h"
#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  TalonSRX fl{6};
  frc::DutyCycleEncoder encoder_fl{6};
  TalonSRX fr{5};
  frc::DutyCycleEncoder encoder_fr{5};
  TalonSRX bl{1};
  frc::DutyCycleEncoder encoder_bl{1};
  TalonSRX br{0};
  frc::DutyCycleEncoder encoder_br{0};
  

  double spdmult = 0.8;
  double topspeed = 1;

  rev::CANSparkMax lshooter{4, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax rshooter{4, rev::CANSparkLowLevel::MotorType::kBrushless};
  rev::CANSparkMax intake{4, rev::CANSparkLowLevel::MotorType::kBrushless};
  

  //Stick Vars
  double lx = 0.0;
  double ly = 0.0;
  double rx = 0.0;
  double ry = 0.0;
  bool LeftStickButton = 0;
  bool RightStickButton = 0;
  
  //A, B, X, Y, Back, and Start

  bool AButton = 0;
  bool BButton = 0;
  bool XButton = 0;
  bool YButton = 0;
  bool StartButton = 0;
  bool BackButton = 0;

  //Trigers and Bumpers

  bool LeftBumper = 0;
  double LeftTrigger = 0;
  bool RightBumper = 0;
  double RightTrigger = 0;
  
 //Misc Vars
  double slider = 0.0;
  double spd = 0.0;
  double direction = 0.0;
  double magnitude = 0.0;
  double turn = 0.0;


  const double Pi = 3.1415926535;

  frc::XboxController xbox{0};
  Pigeon2 gyro{24};

  frc::Field2d m_field;

//Camera and Limelight
  //photon::PhotonCamera pCamera{"Microsoft_Lifecam_HD-3000"};

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  double targetArea = table->GetNumber("ta",0.0);
  double targetSkew = table->GetNumber("ts",0.0);

  frc::Rotation2d getRotation2d;

//Gyro
  //PigeonIMU pGyro {0};
  //double fireAngle= PigeonIMU_BiasedStatus_2_Gyro; 
  // Pigeon is on CANBus (powered from ~12V, and has a device ID of zero
  //PigeonIMU::GeneralStatus genStatus;

  // Set up wheel locations
  frc::Translation2d m_frontLeftLocation{0.53416_m, 0.53416_m};
  frc::Translation2d m_frontRightLocation{0.53416_m, -0.53416_m};
  frc::Translation2d m_backLeftLocation{-0.53416_m, 0.53416_m};
  frc::Translation2d m_backRightLocation{-0.53416_m, -0.53416_m};

// Create kinematics object using the wheel locations.
  frc::MecanumDriveKinematics m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

    frc::MecanumDriveOdometry m_odometry{
    m_kinematics,
    getRotation2d,
      frc::MecanumDriveWheelPositions{
        units::inch_t{(((fr.GetSelectedSensorPosition(0))/4096)*25)},
        units::inch_t{(((fl.GetSelectedSensorPosition(0))/4096)*25)},
        units::inch_t{(((bl.GetSelectedSensorPosition(0))/4096)*25)},
        units::inch_t{(((br.GetSelectedSensorPosition(0))/4096)*25)}
  },
  frc::Pose2d{5_m, 23.5_m, 0_rad}
  };

};
