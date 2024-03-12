// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCommand.h"
#include "RobotContainer.h"
#include "subsystems/ShooterIntake.h"
#include "Constants.h"

IntakeCommand::IntakeCommand(ShooterIntake* pShooterIntake, int wait) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pShooterIntake = pShooterIntake;
  if(wait > 0) m_pWait = new frc2::WaitCommand(units::second_t(wait));
  AddRequirements(m_pShooterIntake);
}

// Called when the command is initially scheduled.
void IntakeCommand::Initialize() {
  
  m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::intakeVelocity);
  if(m_pWait) m_pWait->Schedule();
}

// Called repeatedly when this Command is scheduled to run
void IntakeCommand::Execute() {
  bool intakeSwitchGet = m_pShooterIntake->getIntakeSensorState();
  #ifdef HAVEBEAMBREAK

  bool frontSwitchGet = m_pShooterIntake->getFrontSensorState();
  bool backSwitchGet = m_pShooterIntake->getBackSensorState();

  if(frontSwitchGet == true && intakeSwitchGet == false && backSwitchGet == false){
    m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::slowIntakeVelocity);

  } else if(frontSwitchGet == true && intakeSwitchGet == true && backSwitchGet == false){
    m_pShooterIntake->setIntakeVelocity(0);

  }else if(frontSwitchGet == true && intakeSwitchGet == true && backSwitchGet == true){
    m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::backwardsIntakeVelocity);

  }
  #else
  if(intakeSwitchGet == true){
    m_pShooterIntake->setIntakeVelocity(0);
  }
  #endif
}

// Called once the command ends or is interrupted.
void IntakeCommand::End(bool interrupted) {
  m_pShooterIntake->setIntakeVelocity(0);
}

// Returns true when the command should end.
bool IntakeCommand::IsFinished() {
  #ifdef HAVEBEAMBREAK
  if(m_pShooterIntake->getFrontSensorState() == true 
    && m_pShooterIntake->getIntakeSensorState() == true 
    && m_pShooterIntake->getBackSensorState() == false){
    return true;
  } else {
    return false;
  }
  #else
  return m_pShooterIntake->getIntakeSensorState() || (m_pWait && m_pWait->IsFinished());
  #endif
}
