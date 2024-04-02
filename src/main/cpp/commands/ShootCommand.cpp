// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootCommand.h"
#include "RobotContainer.h"
#include "subsystems/ShooterIntake.h"
#include "Constants.h"

ShootCommand::ShootCommand(
  ShooterIntake* pShooterIntake,
  int wait
) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pShooterIntake = pShooterIntake;
  if(wait > 0) m_pWait = new frc2::WaitCommand(units::second_t(wait));
  AddRequirements(m_pShooterIntake);
}

// Called when the command is initially scheduled.
void ShootCommand::Initialize() {

  m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::intakeShootVelocity);
  std::cout << "shootcommand init/n";
  if(m_pWait) m_pWait->Schedule();

}

// Called repeatedly when this Command is scheduled to run
void ShootCommand::Execute() {}

// Called once the command ends or is interrupted.
void ShootCommand::End(bool interrupted) {
  m_pShooterIntake->setIntakeVelocity(0);
  //std::cout << "shootcommand end/n";
}
  
// Returns true when the command should end.
bool ShootCommand::IsFinished() {
  return m_pWait && m_pWait->IsFinished();
}
