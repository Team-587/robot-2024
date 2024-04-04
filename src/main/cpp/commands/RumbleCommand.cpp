// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RumbleCommand.h"

RumbleCommand::RumbleCommand(
  frc::GenericHID::RumbleType RumbleType,
  units::second_t RumbleOnTime,
  units::second_t RumbleOffTime,
  double RumbleIntensity) :
    m_WaitRumbleOnCommand{RumbleOnTime},
    m_WaitRumbleOffCommand{RumbleOffTime} {
  // Use addRequirements() here to declare subsystem dependencies.
  m_RumbleType = RumbleType;
  m_RumbleIntensity = RumbleIntensity;
}

// Called when the command is initially scheduled.
void RumbleCommand::Initialize() {
  m_WaitRumbleOnCommand.Schedule();
  m_DriverController.SetRumble(m_RumbleType, m_RumbleIntensity);
  m_CodriverController.SetRumble(m_RumbleType, m_RumbleIntensity);
}

// Called repeatedly when this Command is scheduled to run
void RumbleCommand::Execute() {
  if(m_WaitRumbleOnCommand.IsFinished()) {
    m_WaitRumbleOffCommand.Schedule();
    m_DriverController.SetRumble(m_RumbleType, 0);
    m_CodriverController.SetRumble(m_RumbleType, 0);
  }
}

// Called once the command ends or is interrupted.
void RumbleCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool RumbleCommand::IsFinished() {
  return m_WaitRumbleOnCommand.IsFinished() && m_WaitRumbleOffCommand.IsFinished();
}
