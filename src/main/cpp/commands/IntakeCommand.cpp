// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand.h"
#include "RobotContainer.h"
#include "subsystems/ShooterIntake.h"
#include "Constants.h"

IntakeCommand::IntakeCommand(ShooterIntake* pShooterIntake) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pShooterIntake = pShooterIntake;
}

// Called when the command is initially scheduled.
void IntakeCommand::Initialize() {
  
  m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::intakeVelocity);

}

// Called repeatedly when this Command is scheduled to run
void IntakeCommand::Execute() {}

// Called once the command ends or is interrupted.
void IntakeCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool IntakeCommand::IsFinished() {
  return false;
}
