// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/NoteVisionCommand.h"

NoteVisionCommand::NoteVisionCommand(
  NoteVisionSubsystem* pNoteVisionSubsystem, 
  DriveSubsystem* pDriveSubsystem,
  ShooterIntake* pShooterIntake,
  IntakeCommand* pIntakeCommand,
  //frc2::InstantCommand* pStartIntake,
  frc2::InstantCommand* pPickUpPosition,
  frc2::InstantCommand* pHoldPosition
  //frc2::InstantCommand* pStopIntake
  ): m_VisionThread{"Note", pNoteVisionSubsystem} {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pNoteVisionSubsystem = pNoteVisionSubsystem;
  m_pDriveSubsystem = pDriveSubsystem;
  m_pShooterIntake = pShooterIntake;
  m_pIntakeCommand = pIntakeCommand;
  //m_pStartIntake = pStartIntake;
  m_pPickUpPosition = pPickUpPosition;
  m_pHoldPosition = pHoldPosition;
  //m_pStopIntake = pStopIntake;

  AddRequirements(m_pNoteVisionSubsystem);
  AddRequirements(m_pDriveSubsystem);

  //turnController.SetTolerance(3, 5);
  //turnController.SetIntegratorRange(-.05, .05);
  //forwardController.SetTolerance(1, 3);
  //forwardController.SetIntegratorRange(-.05, .05);
}

// Called when the command is initially scheduled.
void NoteVisionCommand::Initialize() {
  m_pPickUpPosition->Schedule();
  //m_pStartIntake->Schedule();
  m_pIntakeCommand->Schedule();
  m_VisionThread.Enable(true);
}


void NoteVisionCommand::Execute() {
  
  //std::cout << "NoteVisionCommand\n";

  //const std::optional<photon::PhotonTrackedTarget> target = m_pNoteVisionSubsystem->GetBestTarget();
  photon::PhotonTrackedTarget *pTarget = m_VisionThread.GetTarget();

  double forwardSpeed = -.2;
  double rotationSpeed = 0;
  //frc::SmartDashboard::PutBoolean("NoteTargets", target.has_value());

  if (pTarget/*target.has_value()*/) {
                        
    //rotationSpeed = -turnController.Calculate(target.value().GetYaw(), 0);
    //rotationSpeed = std::fmin(1, std::fmax(-1, rotationSpeed / 8));
    if (fabs(pTarget->GetYaw()) > 3) {
      rotationSpeed = pTarget->GetYaw() > 0 ? .1 : -.1;
      std::cout << "NoteVisionCommand - Yaw: " << pTarget->GetYaw() << ", Rotation: " << rotationSpeed << "\n";
    }
    double x = m_driverController.GetRightX();
    m_pDriveSubsystem->Drive(
      units::meters_per_second_t{forwardSpeed},
      units::meters_per_second_t{0},
      units::radians_per_second_t{fabs(x) > .1 ? x : rotationSpeed}, false);

  }
}

// Called once the command ends or is interrupted.
void NoteVisionCommand::End(bool interrupted) {
  //m_pStopIntake->Schedule();
  m_pHoldPosition->Schedule();
  m_pIntakeCommand->Cancel();
  m_VisionThread.Enable(false);
}

// Returns true when the command should end.
bool NoteVisionCommand::IsFinished() {
  return m_pShooterIntake->getIntakeSensorState();
}
