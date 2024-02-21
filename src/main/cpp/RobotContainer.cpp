// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DriverStation.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/smartdashboard/Smartdashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/filter/SlewRateLimiter.h>


#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer():
        m_drive{&m_vision},
        m_NoteVisionCommand{&m_NoteVisionSubsystem, &m_drive},
        m_AprilTagVisionCommand{&m_AprilTagVisionSubsystem, &m_drive} {
  // Initialize all of your commands and subsystems here, 
  
  //  the button bindings
  ConfigureButtonBindings();

  pathplanner::NamedCommands::registerCommand("drive stop", frc2::cmd::RunOnce([this] {this->m_drive.Stop();}, {&m_drive}));
  pathplanner::NamedCommands::registerCommand("Short Shoot", frc2::cmd::RunOnce([this] {this->m_robotarm.ArmPosition(RobotArm::ShortShootAngle, RobotArm::ShortShootLength);}, {&m_robotarm}));
  pathplanner::NamedCommands::registerCommand("Long Shoot", frc2::cmd::RunOnce([this] {this->m_robotarm.ArmPosition(RobotArm::LongShootAngle, RobotArm::LongShootLength);}, {&m_robotarm}));
  pathplanner::NamedCommands::registerCommand("Pick Up Position", frc2::cmd::RunOnce([this] {this->m_robotarm.ArmPosition(RobotArm::PickUpAngle, RobotArm::PickUpLength);}, {&m_robotarm}));
  pathplanner::NamedCommands::registerCommand("Hold Position", frc2::cmd::RunOnce([this] {this->m_robotarm.ArmPosition(RobotArm::HoldAngle, RobotArm::HoldLength);}, {&m_robotarm}));
  pathplanner::NamedCommands::registerCommand("Start Intake", frc2::cmd::RunOnce([this] {this->m_shooter.setIntakeStart(); }, {&m_shooter}));
  pathplanner::NamedCommands::registerCommand("Stop Intake", frc2::cmd::RunOnce([this] {this->m_shooter.setIntakeStop(); }, {&m_shooter}));
  pathplanner::NamedCommands::registerCommand("Shoot", frc2::cmd::RunOnce([this] {this->m_shooter.setBeginShooter(); }, {&m_shooter}));
  pathplanner::NamedCommands::registerCommand("Stop Shoot", frc2::cmd::RunOnce([this] {this->m_shooter.setShooterVelocity(ShooterIntake::StopShootVelocity); }, {&m_shooter}));
  pathplanner::NamedCommands::registerCommand("Amp Position", frc2::cmd::RunOnce([this] {this->m_robotarm.ArmPosition(RobotArm::AmpAngle, RobotArm::AmpLength);}, {&m_robotarm}));
  pathplanner::NamedCommands::registerCommand("Short Shoot Velocity", frc2::cmd::RunOnce([this] {this->m_shooter.setShooterVelocity(ShooterIntake::ShortShootVelocity); }, {&m_shooter}));
  pathplanner::NamedCommands::registerCommand("Long Shoot Velocity", frc2::cmd::RunOnce([this] {this->m_shooter.setShooterVelocity(ShooterIntake::LongShootVelocity); }, {&m_shooter}));
  pathplanner::NamedCommands::registerCommand("print hello", frc2::cmd::Print("hello"));

  pathplanner::NamedCommands::registerCommand("Short Shot Setup", frc2::cmd::Sequence(
          frc2::cmd::RunOnce([this] {this->m_robotarm.ArmPosition(RobotArm::ShortShootAngle, RobotArm::ShortShootLength);}, {&m_robotarm}), 
          frc2::cmd::RunOnce([this] {this->m_shooter.setShooterVelocity(ShooterIntake::ShortShootVelocity); }, {&m_shooter})));
  pathplanner::NamedCommands::registerCommand("Shoot + Intake", frc2::cmd::Sequence(
          frc2::cmd::RunOnce([this] {this->m_drive.Stop();}, {&m_drive}),
          frc2::cmd::RunOnce([this] {this->m_shooter.setBeginShooter(); }, {&m_shooter}),
          frc2::cmd::Wait(0.3_s),
          frc2::cmd::RunOnce([this] {this->m_robotarm.ArmPosition(RobotArm::PickUpAngle, RobotArm::PickUpLength);}, {&m_robotarm}),
          frc2::cmd::RunOnce([this] {this->m_shooter.setIntakeStart(); }, {&m_shooter})));
  
  pathplanner::NamedCommands::registerCommand("Long Shot Setup", frc2::cmd::Sequence(
          frc2::cmd::RunOnce([this] {this->m_robotarm.ArmPosition(RobotArm::LongShootAngle, RobotArm::LongShootLength);}, {&m_robotarm}), 
          frc2::cmd::RunOnce([this] {this->m_shooter.setShooterVelocity(ShooterIntake::LongShootVelocity); }, {&m_shooter})));
  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        static frc::SlewRateLimiter<units::velocity::meters_per_second> filterX{2.0_m / 1_s / 1_s};
        static frc::SlewRateLimiter<units::velocity::meters_per_second> filterY{2.0_m / 1_s / 1_s};
        units::velocity::meters_per_second_t jx(m_driverController.GetLeftY());
        units::velocity::meters_per_second_t jy(m_driverController.GetLeftX());

        //units::length::meter jx = filterY.Calculate(m_driverController.GetLeftY());
        m_drive.Drive(
            //jx, 
            //jy,
            filterX.Calculate(jx),
            filterY.Calculate(jy),
            units::radians_per_second_t{m_driverController.GetRightX()}, true);
      },
      {&m_drive}));

  m_tentacle.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_tentacle.moveTentacles(
            m_codriverController.GetLeftY(),
            m_codriverController.GetRightY());
      },
      {&m_tentacle}));

  const std::string AmpNote_Note1_Str = "Amp note + Note 1";
  const std::string Center_Amp_Note1_Str = "Center note + Amp note + Note 1";
  const std::string SourceNote_Note3_Str = "Source Note + Note 3";
  const std::string Rectangle_Str = "Rectangle";
  const std::string Center_Note3_Note4_Str = "Center + Note 3 + Note 4";
  const std::string Center_Note2_Note3_Str = "Center + Note 2 + Note 3";
  AmpNote_Note1 = pathplanner::PathPlannerAuto(AmpNote_Note1_Str).ToPtr().Unwrap();
  Center_Amp_Note1 = pathplanner::PathPlannerAuto(Center_Amp_Note1_Str).ToPtr().Unwrap();
  SourceNote_Note3 = pathplanner::PathPlannerAuto(SourceNote_Note3_Str).ToPtr().Unwrap();
  Rectangle = pathplanner::PathPlannerAuto(Rectangle_Str).ToPtr().Unwrap();
  Center_Note3_Note4 = pathplanner::PathPlannerAuto(Center_Note3_Note4_Str).ToPtr().Unwrap();
  Center_Note2_Note3 = pathplanner::PathPlannerAuto(Center_Note2_Note3_Str).ToPtr().Unwrap();

  m_chooser.SetDefaultOption(AmpNote_Note1_Str, AmpNote_Note1.get());
  m_chooser.AddOption(Center_Amp_Note1_Str, Center_Amp_Note1.get());
  m_chooser.AddOption(SourceNote_Note3_Str, SourceNote_Note3.get());
  m_chooser.AddOption(Center_Note3_Note4_Str, Center_Note3_Note4.get());
  m_chooser.AddOption(Center_Note2_Note3_Str, Center_Note2_Note3.get());
  m_chooser.AddOption(Rectangle_Str, Rectangle.get());

   frc::SmartDashboard::PutData("Auto", &m_chooser);
   
   frc::SmartDashboard::PutNumber("Command Angle", 0);
   frc::SmartDashboard::PutNumber("Command Elevator", 0);
   frc::SmartDashboard::PutNumber("Command Velocity", 0);

 static frc2::InstantCommand m_MoveArm{[this] {m_robotarm.ArmPosition(frc::SmartDashboard::GetNumber("Command Angle", 0), frc::SmartDashboard::GetNumber("Command Elevator", 0));}, {&m_robotarm}};
 static frc2::InstantCommand m_ShooterVelocity{[this] {m_shooter.setShooterVelocity(frc::SmartDashboard::PutNumber("Command Velocity", 0)); }, {&m_shooter}};

   frc::SmartDashboard::PutData("Velocity Cmd", &m_ShooterVelocity);
   frc::SmartDashboard::PutData("Arm Cmd", &m_MoveArm);
}

void RobotContainer::ConfigureButtonBindings() {

    frc2::JoystickButton startButton{&m_driverController, frc::XboxController::Button::kStart};
    startButton.OnTrue(&m_ZeroHeading);

    frc2::JoystickButton aButton(&m_driverController, frc::XboxController::Button::kA);
    aButton.WhileTrue(&m_NoteVisionCommand);
    
    frc2::JoystickButton bButton(&m_driverController, frc::XboxController::Button::kB);
    bButton.WhileTrue(&m_AprilTagVisionCommand);
    
    frc2::JoystickButton rightBumperDriver{&m_driverController, frc::XboxController::Button::kRightBumper};
    rightBumperDriver.OnTrue(&m_Shoot);

    frc2::JoystickButton leftBumperDriver{&m_driverController, frc::XboxController::Button::kLeftBumper};
    leftBumperDriver.OnTrue(&m_StopShoot);

    frc2::JoystickButton startButtonCoDrive{&m_codriverController, frc::XboxController::Button::kStart};
    startButtonCoDrive.OnTrue(&m_StartTentacles);

    frc2::JoystickButton rightBumperCoDrive{&m_codriverController, frc::XboxController::Button::kRightBumper};
    rightBumperCoDrive.OnTrue(&m_PickUpPosition).OnTrue(&m_StartIntake).OnTrue(&m_StopShoot);

    frc2::JoystickButton leftBumperCoDrive{&m_codriverController, frc::XboxController::Button::kLeftBumper};
    leftBumperCoDrive.OnTrue(&m_HoldPosition).OnTrue(&m_StopIntake).OnTrue(&m_StopShoot);

    frc2::JoystickButton aButtonCoDrive{&m_codriverController, frc::XboxController::Button::kA};
    aButtonCoDrive.OnTrue(&m_AmpPosition).OnTrue(&m_ShortShootVelocity);

    frc2::JoystickButton yButtonCoDrive{&m_codriverController, frc::XboxController::Button::kY};
    yButtonCoDrive.OnTrue(&m_ShortShootPosition).OnTrue(&m_ShortShootVelocity);

    frc2::JoystickButton xButtonCoDrive{&m_codriverController, frc::XboxController::Button::kX};
    xButtonCoDrive.OnTrue(&m_LongShootPosition).OnTrue(&m_LongShootVelocity);

    frc2::JoystickButton bButtonCoDrive{&m_codriverController, frc::XboxController::Button::kB};
    bButtonCoDrive.OnTrue(&m_Shoot);

 }

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
}
