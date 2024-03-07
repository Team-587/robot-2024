// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootCommand.h"
#include "RobotContainer.h"
#include "subsystems/ShooterIntake.h"
#include "Constants.h"

ShootCommand::ShootCommand(
  ShooterIntake* pShooterIntake
) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pShooterIntake = pShooterIntake;
}

// Called when the command is initially scheduled.
void ShootCommand::Initialize() {

  m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::intakeShootVelocity);

}

// Called repeatedly when this Command is scheduled to run
void ShootCommand::Execute() {}

// Called once the command ends or is interrupted.
void ShootCommand::End(bool interrupted) {
  m_pShooterIntake->setIntakeVelocity(0);
}
  
// Returns true when the command should end.
bool ShootCommand::IsFinished() {
  return false;
}
