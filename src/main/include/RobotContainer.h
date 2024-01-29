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
#include "subsystems/ShooterIntake.h"
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

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

  // The robot's subsystems and commands are defined here...
  public:
  // The robot's subsystems
  DriveSubsystem m_drive;

  //ShooterIntake m_shooter;
  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();

    frc2::InstantCommand m_ZeroHeading{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
    frc2::InstantCommand m_DriveStop{[this] {m_drive.Stop(); }, {&m_drive}};
  //Setting up commands from the Shooter
  /*
    frc2::InstantCommand m_StartIntake{[this] {m_shooter.setIntakeStart(); }, {&m_shooter}};
    frc2::InstantCommand m_StopIntake{[this] {m_shooter.setIntakeStop(); }, {&m_shooter}};
    frc2::InstantCommand m_BeginShoot{[this] {m_shooter.setBeginShooter(); }, {&m_shooter}};
    frc2::InstantCommand m_StopShoot{[this] {m_shooter.setShooterVelocity(0); }, {&m_shooter}};
    frc2::InstantCommand m_ShortShoot{[this] {m_shooter.setShooterVelocity(0.5); }, {&m_shooter}};
    frc2::InstantCommand m_LongShoot{[this] {m_shooter.setShooterVelocity(1.0); }, {&m_shooter}};
*/

    std::unique_ptr<frc2::Command> AmpNote_Note1;
    std::unique_ptr<frc2::Command> Center_Amp_Note1;
    std::unique_ptr<frc2::Command> SourceNote_Note3;
    std::unique_ptr<frc2::Command> Rectangle;

};
