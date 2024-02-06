// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <frc/XboxController.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/DriverStation.h>


#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  kFrontLeftAbsoluteEncoderPort,
                  kFrontLeftDriveEncoderReversed,
                  kFrontLeftTurningEncoderReversed,
                  "fl_"},

      m_rearLeft{
          kRearLeftDriveMotorPort,       kRearLeftTurningMotorPort,
          kRearLeftAbsoluteEncoderPort,
          kRearLeftDriveEncoderReversed, kRearLeftTurningEncoderReversed, "rl_"},

      m_frontRight{
          kFrontRightDriveMotorPort,       kFrontRightTurningMotorPort,
          kFrontRightAbsoluteEncoderPort,
          kFrontRightDriveEncoderReversed, kFrontRightTurningEncoderReversed, "fr_"},

      m_rearRight{
          kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
          kRearRightAbsoluteEncoderPort,
          kRearRightDriveEncoderReversed, kRearRightTurningEncoderReversed, "rr_"},
      //m_NavX{frc::SPI::Port::kMXP},
      m_odometry{kDriveKinematics, 
                 m_NavX.GetRotation2d(), 
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d()}, 
      initialPitch{0},
      initialRoll{0},
      currentPitch{0},
      currentRoll{0},
      currentYaw{0},
      imuValid{false}
{
  // Configure the AutoBuilder last
    pathplanner::AutoBuilder::configureHolonomic(
        [this](){ return GetPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
       pathplanner::HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            5.0_mps, // Max module speed, in m/s
            0.3_m, // Drive base radius in meters. Distance from robot center to furthest module.
            NULL //pathplanner::ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance =frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );
   //std::cout << "Drive constuctor positions (" << (double)m_frontLeft.GetPosition().distance << "," 
   //          << (double)m_frontRight.GetPosition().distance << "," << (double)m_rearLeft.GetPosition().distance 
   //          << "," << (double)m_rearRight.GetPosition().distance << ")";
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  //std::cout << "Drive Periodic positions (" << (double)m_frontLeft.GetPosition().distance << "," 
  //          << (double)m_frontRight.GetPosition().distance << "," << (double)m_rearLeft.GetPosition().distance 
  //          << "," << (double)m_rearRight.GetPosition().distance << ")";
  

  m_odometry.Update(m_NavX.GetRotation2d(), //frc::Rotation2d(GetHeading()), 
                    {m_frontLeft.GetPosition(),m_frontRight.GetPosition(), m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
  if (imuValid == false) {
    if (m_NavX.IsCalibrating() == false) {
      initialPitch = m_NavX.GetPitch();
      initialRoll = m_NavX.GetRoll();
      imuValid = true;
      frc::SmartDashboard::PutNumber("InitialPitch", initialPitch);
      frc::SmartDashboard::PutNumber("InitialRoll", initialRoll);
    }
  }

  frc::Pose2d tmpPose = GetPose();
  auto alliance =frc::DriverStation::GetAlliance();
  frc::SmartDashboard::PutNumber("PoseX", (double)tmpPose.X());
  frc::SmartDashboard::PutNumber("PoseY", (double)tmpPose.Y());
  frc::SmartDashboard::PutNumber("PoseRotation", (double)tmpPose.Rotation().Degrees());
  double fl_dist = (double)m_frontLeft.GetPosition().distance;
  frc::SmartDashboard::PutNumber("FL_pos", fl_dist);
  frc::SmartDashboard::PutNumber("FL_angle", (double)m_frontLeft.GetPosition().angle.Degrees());
  frc::SmartDashboard::PutBoolean("Alliance", alliance.value() != frc::DriverStation::Alliance::kRed);

/*static int skip = 0;
skip++;
if (skip >= 50)
{
   skip = 0;
   frc::Pose2d tmpPose = GetPose();
   std::cout << "Periodic Odometry X:" << (double)tmpPose.X() << " Y:" << (double)tmpPose.Y() << " Rot:" << (double)tmpPose.Rotation().Degrees() << "\n";
   std::cout << "Drive Periodic " << (double)GetHeading() << " positions (" << (double)m_frontLeft.GetPosition().distance << "," 
            << (double)m_frontRight.GetPosition().distance << "," << (double)m_rearLeft.GetPosition().distance 
            << "," << (double)m_rearRight.GetPosition().distance << ") ";
    std::cout << "angles(" << (double)m_frontLeft.GetPosition().angle.Degrees() << "," << (double)m_frontRight.GetPosition().angle.Degrees() << ","
            << (double)m_rearLeft.GetPosition().angle.Degrees() << "," << (double)m_rearRight.GetPosition().angle.Degrees() << ")\n";
}
*/
currentPitch = m_NavX.GetPitch();
currentRoll = m_NavX.GetRoll();
currentYaw = -m_NavX.GetYaw();

frc::SmartDashboard::PutNumber("Speed", m_fullSpeed); 
frc::SmartDashboard::PutNumber("Heading", (double)m_NavX.GetRotation2d().Degrees());
frc::SmartDashboard::PutNumber("Pitch", currentPitch); 
frc::SmartDashboard::PutNumber("Yaw", currentYaw);
frc::SmartDashboard::PutNumber("Roll", currentRoll);
}

void DriveSubsystem::driveRobotRelative(const frc::ChassisSpeeds& robotRelativeSpeeds){
    frc::ChassisSpeeds targetSpeeds = frc::ChassisSpeeds::Discretize(robotRelativeSpeeds, 0.02_s);
    targetSpeeds.vx = -targetSpeeds.vx;
    targetSpeeds.vy = -targetSpeeds.vy;
    targetSpeeds.omega = -targetSpeeds.omega;

    //if (frc::DriverStation::IsAutonomousEnabled()) {
        //std::cout << "auto drive speed x:" << (double)targetSpeeds.vx << " y:" << (double)targetSpeeds.vy << " ang:" << (double)targetSpeeds.omega << "\n";
    //}
    
    auto targetStates = kDriveKinematics.ToSwerveModuleStates(targetSpeeds);
    SetModuleStates(targetStates);
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  
  
  if ((double)xSpeed < 0.1 && (double)xSpeed > -0.1){
    xSpeed = (units::meters_per_second_t)0.0;
  }
  if ((double)ySpeed < 0.1 && (double)ySpeed > -0.1){
    ySpeed = (units::meters_per_second_t)0.0;
  }
  if ((double)rot < 0.1 && (double)rot > -0.1){
    rot = (units::radians_per_second_t)0.0;
  }

   // xSpeed = xSpeed * m_fullSpeed;
   // ySpeed = ySpeed * m_fullSpeed;
   // rot = rot * m_fullSpeed;

  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  double rightTriggerValue = (m_driverController.GetRightTriggerAxis() * -.8) + 1.0;

  xSpeed = xSpeed * rightTriggerValue;
  ySpeed = ySpeed * rightTriggerValue;
  rot = rot * rightTriggerValue;
   


  frc::SmartDashboard::PutNumber("xSpeed", (double)xSpeed);  
  frc::SmartDashboard::PutNumber("ySpeed", (double)ySpeed);
  frc::SmartDashboard::PutNumber("Rotation", (double)rot);   
                       
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_NavX.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  /*
  std::cout << "Angles (" << (double)states[0].angle.Degrees() << "," 
          << (double)states[1].angle.Degrees() << "," 
          << (double)states[2].angle.Degrees() << ","
          << (double)states[3].angle.Degrees() 
          << ") Speeds (" << (double)states[0].speed << ","
          << (double)states[1].speed << ","
          << (double)states[2].speed << ","
          << (double)states[3].speed << ")\n";
          */


  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {

/*std::cout << "Angles (" << (double)desiredStates[0].angle.Degrees() << "," 
          << (double)desiredStates[1].angle.Degrees() << "," 
          << (double)desiredStates[2].angle.Degrees() << ","
          << (double)desiredStates[3].angle.Degrees() 
          << ") Speeds (" << (double)desiredStates[0].speed << ","
          << (double)desiredStates[0].speed << ","
          << (double)desiredStates[0].speed << ","
          << (double)desiredStates[0].speed << ")";*/

  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,  AutoConstants::kMaxSpeed);
  double kmaxspeed = (double)AutoConstants::kMaxSpeed;
  desiredStates[0].speed = desiredStates[0].speed / kmaxspeed;
  desiredStates[1].speed = desiredStates[1].speed / kmaxspeed;
  desiredStates[2].speed = desiredStates[2].speed / kmaxspeed;
  desiredStates[3].speed = desiredStates[3].speed / kmaxspeed;

  /*std::cout << "Angles (" << (double)desiredStates[0].angle.Degrees() << "," 
          << (double)desiredStates[1].angle.Degrees() << "," 
          << (double)desiredStates[2].angle.Degrees() << ","
          << (double)desiredStates[3].angle.Degrees() 
          << ") Speeds (" << (double)desiredStates[0].speed << ","
          << (double)desiredStates[0].speed << ","
          << (double)desiredStates[0].speed << ","
          << (double)desiredStates[0].speed << ")\n";*/

  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  std::cout << "Reset Encoders \n";
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

/*
units::degree_t DriveSubsystem::GetHeading() const {
  return m_NavX.GetRotation2d().Degrees();
}
*/

double DriveSubsystem::getPitch() {
  return currentPitch;
}
double DriveSubsystem::getInitialPitch() {
  return initialPitch;
}

double DriveSubsystem::getCurrentYaw() {
  return currentYaw;
}
void DriveSubsystem::ZeroHeading() {
    std::cout << "Zero Heading\n";
  m_NavX.Reset();
  ResetEncoders();
  frc::Pose2d tmpPose{};
  ResetOdometry(tmpPose);
  //m_NavX.ZeroYaw();
}

void DriveSubsystem::Stop() {
  std::cout << "Stop \n";
  m_frontLeft.Stop();
  m_rearLeft.Stop();
  m_frontRight.Stop();
  m_rearRight.Stop();
}

double DriveSubsystem::GetTurnRate() {
  return -m_NavX.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  frc::Pose2d tmpPose = m_odometry.GetPose();

  //if (frc::DriverStation::IsAutonomousEnabled()) {
    //std::cout << "getPose X:" << (double)tmpPose.X() << " Y:" << (double)tmpPose.Y() << " Rot:" << (double)tmpPose.Rotation().Radians() << "\n";
  //}

  return tmpPose;
}

void DriveSubsystem::limitSpeed(double speed) {
  m_fullSpeed = speed;
}

void DriveSubsystem::fullSpeed() {
  m_fullSpeed = 1.0;
}
void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {

    frc::Pose2d tmpPose = GetPose();
    std::cout << "Reset Odometry start X:" << (double)tmpPose.X() << " Y:" << (double)tmpPose.Y() << " Rot:" << (double)tmpPose.Rotation().Degrees() << "\n";
    std::cout << "Reset Odometry set   X:" << (double)pose.X() << " Y:" << (double)pose.Y() << " Rot:" << (double)pose.Rotation().Degrees() << "\n";
    m_odometry.ResetPosition(
      m_NavX.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);

    tmpPose = GetPose();
    std::cout << "Reset Odometry after X:" << (double)tmpPose.X() << " Y:" << (double)tmpPose.Y() << " Rot:" << (double)tmpPose.Rotation().Degrees() << "\n";
 
}
