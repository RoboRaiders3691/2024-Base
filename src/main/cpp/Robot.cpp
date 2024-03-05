// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
//#include "LimelightHelpers.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutData("Field", &m_field);

  
  fr.ConfigFactoryDefault();
  fl.ConfigFactoryDefault();
  br.ConfigFactoryDefault();
  bl.ConfigFactoryDefault();

  fr.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::PulseWidthEncodedPosition, 0, 50);
  fl.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::PulseWidthEncodedPosition, 0, 50);
  br.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::PulseWidthEncodedPosition, 0, 50);
  bl.ConfigSelectedFeedbackSensor(TalonSRXFeedbackDevice::PulseWidthEncodedPosition, 0, 50);

  fr.SetSensorPhase(false);
  fl.SetSensorPhase(false);
  br.SetSensorPhase(false);
  bl.SetSensorPhase(false);

  fr.SetSelectedSensorPosition(0,0,10);
  fl.SetSelectedSensorPosition(0,0,10);
  br.SetSelectedSensorPosition(0,0,10);
  bl.SetSelectedSensorPosition(0,0,10);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
// Set up wheel locations
  frc::Translation2d m_frontLeftLocation{0.53416_m, 0.53416_m};
  frc::Translation2d m_frontRightLocation{0.53416_m, -0.53416_m};
  frc::Translation2d m_backLeftLocation{-0.53416_m, 0.53416_m};
  frc::Translation2d m_backRightLocation{-0.53416_m, -0.53416_m};

// Create kinematics object using the wheel locations.
  frc::MecanumDriveKinematics m_kinematics{ m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

// Creating my odometry object from the kinematics object. Here,
// our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing forward.

  frc::MecanumDrivePoseEstimator m_poseEstimator{
      m_kinematics,
      getRotation2d,
      frc::MecanumDriveWheelPositions{
          units::meter_t{encoder_fl.GetDistance()},
          units::meter_t{encoder_fr.GetDistance()},
          units::meter_t{encoder_bl.GetDistance()},
          units::meter_t{encoder_br.GetDistance()}
      },
      frc::Pose2d{5_m, 13.5_m, 0_rad},
      {0.01, 0.01, 0.01},
      {0.1, 0.1, 0.1}
    };
    
    m_poseEstimator.Update(
      getRotation2d,
      frc::MecanumDriveWheelPositions{
        units::meter_t{encoder_fl.GetDistance()},
        units::meter_t{encoder_fr.GetDistance()},
        units::meter_t{encoder_bl.GetDistance()},
        units::meter_t{encoder_br.GetDistance()}
      }
    );
    
    
    // Compute the robot's field-relative position exclusively from vision

  // measurements.

  /*frc::Pose3d visionMeasurement3d = ObjectToRobotPose(

      m_objectInField, m_robotToCamera, m_cameraToObjectEntryRef
      );*/


  // Convert robot's pose from Pose3d to Pose2d needed to apply vision

  // measurements.

  //frc::Pose2d visionMeasurement2d = visionMeasurement3d.ToPose2d();


  // Apply vision measurements. For simulation purposes only, we don't input a

  // latency delay -- on a real robot, this must be calculated based either on

  // known latency or timestamps.

  /*m_poseEstimator.AddVisionMeasurement(visionMeasurement2d,

                                       frc::Timer::GetFPGATimestamp());*/
  frc::Pose2d getpose;
  m_field.SetRobotPose(getpose);

  //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("<variablename>",0.0);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {

  lx = xbox.GetLeftX();
  ly = xbox.GetLeftY();
  rx = xbox.GetRightX();

  AButton = xbox.GetAButton();
  BButton = xbox.GetBButton();
  XButton = xbox.GetXButton();
  YButton = xbox.GetYButton();

  LeftTrigger = xbox.GetLeftTriggerAxis();
  LeftBumper = xbox.GetLeftBumper();
  RightTrigger = xbox.GetRightTriggerAxis();
  RightBumper = xbox.GetRightBumper();


  direction = atan2(ly,lx);
  magnitude = hypot(lx,ly);
  turn = rx*0.7;
  if(magnitude>=1){
    magnitude = 1;
  }
  if(lx>topspeed){
    lx = topspeed;
  }else if(lx<-topspeed){
    lx = -topspeed;
  }
  if(ly>topspeed){
    ly = topspeed;
  }else if(ly<-topspeed){
    ly = -topspeed;
  }
  if(rx>topspeed){
    rx = topspeed;
  }else if(rx<-topspeed){
    rx = -topspeed;
  }

  spd = (spdmult);
  
  //frc::Field2d::InitSendable(&m_field);

  //frc::Field2d::NTSendable(&m_field);

  //frc::Field2d::SetRobotPose(5_m, 13.5_m, 0_rad);
  //frc::SmartDashboard::PutData("Position", getpose);
  frc::SmartDashboard::PutNumber("Direction", direction);
  frc::SmartDashboard::PutData("Field", &m_field);
  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0);
  
  //frc::Field2d::SetRobotPose()
  //frc::SmartDashboard::PutData(&m_field);
  //frc::SmartDashboard::UpdateValues();
  //frc::SmartDashboard::PutData();

  if(ly>=0.1 || ly<=-0.1 || lx>=0.1 ||lx<=-0.1){
    fl.Set(ControlMode::PercentOutput, -spd*((sin(-direction+(.25*Pi)))*magnitude - turn));
    fr.Set(ControlMode::PercentOutput, spd*((sin(-direction-(.25*Pi)))*magnitude + turn));
    bl.Set(ControlMode::PercentOutput, -spd*((sin(-direction-(.25*Pi)))*magnitude - turn));
    br.Set(ControlMode::PercentOutput, spd*((sin(-direction+(.25*Pi)))*magnitude + turn));
  }else if(rx>0.1 || rx<-0.1){
    fl.Set(ControlMode::PercentOutput, spd*turn);
    fr.Set(ControlMode::PercentOutput, spd*turn);
    bl.Set(ControlMode::PercentOutput, spd*turn);
    br.Set(ControlMode::PercentOutput, spd*turn);
  }else{
    direction = 0;
    fl.Set(ControlMode::PercentOutput, 0);
    fr.Set(ControlMode::PercentOutput, 0);
    bl.Set(ControlMode::PercentOutput, 0);
    br.Set(ControlMode::PercentOutput, 0);
  }


  if(AButton){
  //photon::PhotonPipelineResult result = pCamera.GetLatestResult();

/*  double Yaw = result.GetBestTarget().GetYaw();

  frc::SmartDashboard::PutNumber("Robot Yaw", Yaw);

  double Pitch = result.GetBestTarget().GetPitch();

  frc::SmartDashboard::PutNumber("Robot Pitch", Pitch);

  double Area = result.GetBestTarget().GetArea();

  frc::SmartDashboard::PutNumber("Robot Skew", Area);
*/

  /*photon::PhotonTrackedTarget target = result.GetBestTarget();

  int targetID = target.GetFiducialId();

  frc::SmartDashboard::PutNumber("TargetID", targetID);

  double poseAmbiguity = target.GetPoseAmbiguity();

  frc::SmartDashboard::PutNumber("Pose Ambiguity", poseAmbiguity);

  frc::Transform3d bestCameraToTarget = target.GetBestCameraToTarget();

  

  double Xdistance = bestCameraToTarget.X().value()*3.28084;

  double Ydistance = bestCameraToTarget.Y().value()*3.28084;

  double Zdistance = bestCameraToTarget.Z().value()*3.28084;

  frc::SmartDashboard::PutNumber("Xdistance", Xdistance);

  frc::SmartDashboard::PutNumber("Ydistance", Ydistance);
  
  frc::SmartDashboard::PutNumber("Zdistance", Zdistance);

  if(Xdistance > 1.5) {
  fl.Set(ControlMode::PercentOutput, -spd * (.3));
  bl.Set(ControlMode::PercentOutput, -spd * (.3));

  fr.Set(ControlMode::PercentOutput, spd * (.3));
  br.Set(ControlMode::PercentOutput, spd * (.3));
  }

  Xdistance = 0;
  Ydistance = 0;
  Zdistance = 0;*/
 
}




}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
