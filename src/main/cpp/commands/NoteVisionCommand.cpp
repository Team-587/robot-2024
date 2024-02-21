// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NoteVisionCommand.h"

NoteVisionCommand::NoteVisionCommand(NoteVisionSubsystem* pNoteVisionSubsystem, DriveSubsystem* pDriveSubsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pNoteVisionSubsystem = pNoteVisionSubsystem;
  m_pDriveSubsystem = pDriveSubsystem;

  AddRequirements(m_pNoteVisionSubsystem);
  AddRequirements(m_pDriveSubsystem);

  turnController.SetTolerance(3, 5);
  turnController.SetIntegratorRange(-.05, .05);
  forwardController.SetTolerance(1, 3);
  forwardController.SetIntegratorRange(-.05, .05);
}

// Called when the command is initially scheduled.
void NoteVisionCommand::Initialize() {}


void NoteVisionCommand::Execute() {
  
  //std::cout << "NoteVisionCommand\n";

  const auto& result = m_pNoteVisionSubsystem->GetCamera()->GetLatestResult();

  forwardSpeed = -.2;
  frc::SmartDashboard::PutBoolean("NoteTargets", result.HasTargets());

  if (result.HasTargets()) {
                        
    rotationSpeed = -turnController.Calculate(result.GetBestTarget().GetYaw(), 0);
    rotationSpeed = std::fmin(1, std::fmax(-1, rotationSpeed / 5));

    m_pDriveSubsystem->Drive(
      units::meters_per_second_t{forwardSpeed},
      units::meters_per_second_t{0},
      units::radians_per_second_t{rotationSpeed}, true);

  } else {
    rotationSpeed = 0;
  }
}

// Called once the command ends or is interrupted.
void NoteVisionCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool NoteVisionCommand::IsFinished() {
  
  //check the note sensor to end

  return false;
}
