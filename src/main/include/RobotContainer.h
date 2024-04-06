// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc/DriverStation.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "Vision.h"
#include "subsystems/NoteVisionSubsystem.h"
#include "subsystems/AprilTagVisionSubsystem.h"
#include "commands/NoteVisionCommand.h"
#include "commands/AprilTagVisionCommand.h"
#include "commands/ShootCommand.h"
#include "subsystems/ShooterIntake.h"
#include "subsystems/RobotArm.h"
#include "subsystems/TentaclesSubsystem.h"
#include "subsystems/Lights.h"
#include "commands/IntakeCommand.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  
  frc2::Command* GetAutonomousCommand();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  
  frc::XboxController m_codriverController{OIConstants::kCoDriverControllerPort};

  // The robot's subsystems and commands are defined here...
  public:
  // The robot's subsystems
  DriveSubsystem m_drive;
  RobotArm m_robotarm;
  ShooterIntake m_shooter;
  Vision m_vision;
  NoteVisionSubsystem m_NoteVisionSubsystem;
  AprilTagVisionSubsystem m_AprilTagVisionSubsystem;
  NoteVisionCommand m_NoteVisionCommand;
  AprilTagVisionCommand m_AprilTagVisionCommand;
  IntakeCommand m_IntakeCommand;
  ShootCommand m_ShootCommand;

  TentaclesSubsystem m_tentacle;

  Lights m_lights;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();

    frc2::InstantCommand m_ZeroHeading{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
    frc2::InstantCommand m_StartTentacles{[this] {m_tentacle.allowTentacleExtend();}, {&m_tentacle}};
    frc2::InstantCommand m_StartEndgame{[this] {m_shooter.setEndgame(); }, {&m_shooter}};
    frc2::InstantCommand m_DriveStop{[this] {m_drive.Stop(); }, {&m_drive}};
    
  //Setting up commands from the Shooter

    frc2::InstantCommand m_StartIntake{[this] {m_shooter.setIntakeStart(); }, {&m_shooter}};
    //frc2::InstantCommand m_StartIntakeShoot{[this] {m_shooter.setIntakeShoot(); }, {&m_shooter}};
    frc2::InstantCommand m_StopIntake{[this] {m_shooter.setIntakeStop(); }, {&m_shooter}};
    frc2::InstantCommand m_Shoot{[this] {m_shooter.setBeginShooter(); }, {&m_shooter}};
    frc2::InstantCommand m_StopShoot{[this] {m_shooter.setShooterVelocity(ShooterIntake::StopShootVelocity); }, {&m_shooter}};
    frc2::InstantCommand m_ShortShootVelocity{[this] {m_shooter.setShooterVelocity(ShooterIntake::ShortShootVelocity); }, {&m_shooter}};
    frc2::InstantCommand m_LongShootVelocity{[this] {m_shooter.setShooterVelocity(ShooterIntake::LongShootVelocity); }, {&m_shooter}};
    frc2::InstantCommand m_PickUpPosition{[this] {m_robotarm.ArmPosition(RobotArm::PickUpAngle, RobotArm::PickUpLength);}, {&m_robotarm}};
    frc2::InstantCommand m_HoldPosition{[this] {m_robotarm.ArmPosition(RobotArm::HoldAngle, RobotArm::HoldLength);}, {&m_robotarm}};
    frc2::InstantCommand m_AmpPosition{[this] {m_robotarm.ArmPosition(RobotArm::AmpAngle, RobotArm::AmpLength);}, {&m_robotarm}};
    frc2::InstantCommand m_ShortShootPosition{[this] {m_robotarm.ArmPosition(RobotArm::ShortShootAngle, RobotArm::ShortShootLength);}, {&m_robotarm}};
    frc2::InstantCommand m_LongShootPosition{[this] {m_robotarm.ArmPosition(RobotArm::LongShootAngle, RobotArm::LongShootLength);}, {&m_robotarm}}; 
    frc2::InstantCommand m_trapPosition{[this] {m_robotarm.ArmPosition(RobotArm::PickUpAngle, RobotArm::PickUpLength);}, {&m_robotarm}};
    frc2::InstantCommand m_trapShootVelocity{[this] {m_shooter.setShooterVelocity(0.35); }, {&m_shooter}};
    frc2::InstantCommand m_AmpShootVelocity{[this] {m_shooter.setShooterVelocity(ShooterIntake::AmpShootVelocity);}, {&m_shooter}};
    frc2::InstantCommand m_FeedShootVelocity{[this] {m_shooter.setShooterVelocity(ShooterIntake::FeedShootVelocity);}, {&m_shooter}};
    frc2::InstantCommand m_TrapShootPosition{[this] {m_robotarm.ArmPosition(RobotArm::TrapShootAngle, RobotArm::PickUpLength);}, {&m_robotarm}};
    frc2::InstantCommand m_TrapShootVelocity{[this] {m_shooter.setShooterVelocity(ShooterIntake::TrapShootVelocity);}, {&m_shooter}};
    frc2::WaitCommand m_wait{3_s};

    frc2::SequentialCommandGroup m_LongSetup {
      m_LongShootPosition,
      m_LongShootVelocity
    };

    std::unique_ptr<frc2::Command> AmpNote_Note1;
    std::unique_ptr<frc2::Command> Center_Amp_Note1;
    std::unique_ptr<frc2::Command> SourceNote_Note3;
    std::unique_ptr<frc2::Command> FourPiece;
    std::unique_ptr<frc2::Command> Center_Note3_Note4;
    std::unique_ptr<frc2::Command> Center_Note2_Note3;
    std::unique_ptr<frc2::Command> Source_Note5;
    std::unique_ptr<frc2::Command> Amp_Move;
    std::unique_ptr<frc2::Command> Feeder;
    std::unique_ptr<frc2::Command> Amp;
    std::unique_ptr<frc2::Command> Destroy;
    std::unique_ptr<frc2::Command> Amp_Note1_Note2;
    std::unique_ptr<frc2::Command> RedCenterFivePiece;
    std::unique_ptr<frc2::Command> RedSourceMidline;
    std::unique_ptr<frc2::Command> RedAmpMidline;
    std::unique_ptr<frc2::Command> BlueCenterFivePiece;
    std::unique_ptr<frc2::Command> BlueSourceMidline;
    //std::unique_ptr<frc2::Command> BlueAmpMidline;   

};
