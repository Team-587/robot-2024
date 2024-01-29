// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
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


#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  //  the button bindings
  ConfigureButtonBindings();

  pathplanner::NamedCommands::registerCommand("drive stop", frc2::cmd::RunOnce([this] {this->m_drive.Stop();}, {&m_drive}));
  pathplanner::NamedCommands::registerCommand("print hello", frc2::cmd::Print("hello"));
  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        m_drive.Drive(
            units::meters_per_second_t{m_driverController.GetLeftY()},
            units::meters_per_second_t{m_driverController.GetLeftX()},
            units::radians_per_second_t{m_driverController.GetRightX()}, true);
      },
      {&m_drive}));

    const std::string AmpNote_Note1_Str = "Amp note + Note 1\n";
    const std::string Center_Amp_Note1_Str = "Center note + Amp note + Note 1\n";
    const std::string SourceNote_Note3_Str = "Source Note + Note 3\n";
    const std::string Rectangle_Str = "Rectangle\n";
    AmpNote_Note1 = pathplanner::PathPlannerAuto("Amp note + Note 1").ToPtr().Unwrap();
    Center_Amp_Note1 = pathplanner::PathPlannerAuto("Center note + Amp note + Note 1").ToPtr().Unwrap();
    SourceNote_Note3 = pathplanner::PathPlannerAuto("Source Note + Note 3").ToPtr().Unwrap();
    Rectangle = pathplanner::PathPlannerAuto("Rectangle").ToPtr().Unwrap();

    m_chooser.SetDefaultOption(AmpNote_Note1_Str, AmpNote_Note1.get());
    m_chooser.AddOption(Center_Amp_Note1_Str, Center_Amp_Note1.get());
    m_chooser.AddOption(SourceNote_Note3_Str, SourceNote_Note3.get());
    m_chooser.AddOption(Rectangle_Str, Rectangle.get());

       frc::SmartDashboard::PutData("Auto", &m_chooser);
}

void RobotContainer::ConfigureButtonBindings() {

    frc2::JoystickButton startButton{&m_driverController, frc::XboxController::Button::kStart};
    startButton.OnTrue(&m_ZeroHeading);
 }

frc2::Command* RobotContainer::GetAutonomousCommand() {
return m_chooser.GetSelected();
}
