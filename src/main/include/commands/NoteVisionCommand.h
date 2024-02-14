// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/NoteVisionSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class NoteVisionCommand
    : public frc2::CommandHelper<frc2::Command, NoteVisionCommand> {
 public:
  NoteVisionCommand(NoteVisionSubsystem* pNoteVisionSubsystem, DriveSubsystem* pDriveSubsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    NoteVisionSubsystem* m_pNoteVisionSubsystem;
    DriveSubsystem* m_pDriveSubsystem;
    frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
    frc::PIDController forwardController{VisionConstants::VISION_LINEAR_P, 0.0, VisionConstants::VISION_LINEAR_D};
    frc::PIDController turnController{VisionConstants::VISION_ANGULAR_P, 0.0, VisionConstants::VISION_ANGULAR_D};

    double rotationSpeed;
    double forwardSpeed;
};
