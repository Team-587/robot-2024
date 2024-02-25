// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <frc/XboxController.h>
#include <frc/DriverStation.h>

#include "subsystems/AprilTagVisionSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterIntake.h"
#include "subsystems/VisionSubsystem.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AprilTagVisionCommand
    : public frc2::CommandHelper<frc2::Command, AprilTagVisionCommand> {
 public:

  AprilTagVisionCommand(
    AprilTagVisionSubsystem* pAprilTagVisionSubsystem, 
    DriveSubsystem* pDriveSubsystem,
    ShooterIntake* pShooterIntake,
    frc2::InstantCommand* pStartIntake,
    frc2::InstantCommand* pPickUpPosition,
    frc2::InstantCommand* pHoldPosition,
    frc2::InstantCommand* pStopIntake);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    AprilTagVisionSubsystem* m_pAprilTagVisionSubsystem;
    DriveSubsystem* m_pDriveSubsystem;
    ShooterIntake* m_pShooterIntake;
    frc2::InstantCommand* m_pStartIntake;
    frc2::InstantCommand* m_pPickUpPosition;
    frc2::InstantCommand* m_pHoldPosition;
    frc2::InstantCommand* m_pStopIntake;

    frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

    frc::PIDController forwardController{VisionConstants::VISION_LINEAR_P, 0.0, VisionConstants::VISION_LINEAR_D};
    frc::PIDController turnController{VisionConstants::VISION_ANGULAR_P, 0.0, VisionConstants::VISION_ANGULAR_D};

    double rotationSpeed;
    double forwardSpeed;

    int aprilTagID;

    std::optional<frc::DriverStation::Alliance> alliance;
};
