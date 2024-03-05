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

  TalonSRX fl{13};
  frc::DutyCycleEncoder encoder_fl{13};
  TalonSRX fr{10};
  frc::DutyCycleEncoder encoder_fr{10};
  TalonSRX bl{11};
  frc::DutyCycleEncoder encoder_bl{11};
  TalonSRX br{12};
  frc::DutyCycleEncoder encoder_br{12};

  double spdmult = 0.8;
  double topspeed = 1;
  


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
  Pigeon2 gyro{4};

  frc::Field2d m_field;

//Camera and Limelight
  //photon::PhotonCamera pCamera{"Microsoft_Lifecam_HD-3000"};

  //std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  //double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  //double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  //double targetArea = table->GetNumber("ta",0.0);
  //double targetSkew = table->GetNumber("ts",0.0);

  frc::Rotation2d getRotation2d;

//Gyro
  PigeonIMU pGyro {0};
  double fireAngle= PigeonIMU_BiasedStatus_2_Gyro; 
  // Pigeon is on CANBus (powered from ~12V, and has a device ID of zero
  PigeonIMU::GeneralStatus genStatus;
};
