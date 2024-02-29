// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NoteVisionCommand.h"

NoteVisionCommand::NoteVisionCommand(
  NoteVisionSubsystem* pNoteVisionSubsystem, 
  DriveSubsystem* pDriveSubsystem,
  ShooterIntake* pShooterIntake,
  frc2::InstantCommand* pStartIntake,
  frc2::InstantCommand* pPickUpPosition,
  frc2::InstantCommand* pHoldPosition,
  frc2::InstantCommand* pStopIntake) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pNoteVisionSubsystem = pNoteVisionSubsystem;
  m_pDriveSubsystem = pDriveSubsystem;
  m_pShooterIntake = pShooterIntake;
  m_pStartIntake = pStartIntake;
  m_pPickUpPosition = pPickUpPosition;
  m_pHoldPosition = pHoldPosition;
  m_pStopIntake = pStopIntake;

  AddRequirements(m_pNoteVisionSubsystem);
  AddRequirements(m_pDriveSubsystem);

  turnController.SetTolerance(3, 5);
  turnController.SetIntegratorRange(-.05, .05);
  forwardController.SetTolerance(1, 3);
  forwardController.SetIntegratorRange(-.05, .05);
}

// Called when the command is initially scheduled.
void NoteVisionCommand::Initialize() {
  m_pPickUpPosition->Schedule();
  m_pStartIntake->Schedule();
}


void NoteVisionCommand::Execute() {
  
  //std::cout << "NoteVisionCommand\n";

  const std::optional<photon::PhotonTrackedTarget> target = m_pNoteVisionSubsystem->GetBestTarget();

  forwardSpeed = .1;
  frc::SmartDashboard::PutBoolean("NoteTargets", target.has_value());

  if (target.has_value()) {
                        
    rotationSpeed = -turnController.Calculate(target.value().GetYaw(), 0);
    rotationSpeed = std::fmin(1, std::fmax(-1, rotationSpeed / 8));

    m_pDriveSubsystem->Drive(
      units::meters_per_second_t{forwardSpeed},
      units::meters_per_second_t{0},
      units::radians_per_second_t{rotationSpeed}, true);

  } else {
    rotationSpeed = 0;
  }
}

// Called once the command ends or is interrupted.
void NoteVisionCommand::End(bool interrupted) {
  m_pStopIntake->Schedule();
  m_pHoldPosition->Schedule();
}

// Returns true when the command should end.
bool NoteVisionCommand::IsFinished() {
  return m_pShooterIntake->getIntakeSensorState();
}
