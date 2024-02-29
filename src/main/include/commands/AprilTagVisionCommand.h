// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/DriverStation.h>

#include "subsystems/AprilTagVisionSubsystem.h"
#include "subsystems/DriveSubsystem.h"
<<<<<<< Updated upstream
=======
#include "subsystems/ShooterIntake.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/DistanceBuckets.h"
#include "subsystems/RobotArm.h"

>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
  AprilTagVisionCommand(AprilTagVisionSubsystem* pAprilTagVisionSubsystem, DriveSubsystem* pDriveSubsystem);
=======
  AprilTagVisionCommand(
    AprilTagVisionSubsystem* pAprilTagVisionSubsystem, 
    DriveSubsystem* pDriveSubsystem,
    ShooterIntake* pShooterIntake,
    RobotArm* pRobotArm);

   std::optional<DistanceBuckets> GetDistances(units::meter_t distance);
>>>>>>> Stashed changes

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    AprilTagVisionSubsystem* m_pAprilTagVisionSubsystem;
    DriveSubsystem* m_pDriveSubsystem;
<<<<<<< Updated upstream
=======
    ShooterIntake* m_pShooterIntake;
    RobotArm* m_pRobotArm;
    DistanceBuckets distArray[10];

>>>>>>> Stashed changes
    frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
    frc::PIDController forwardController{VisionConstants::VISION_LINEAR_P, 0.0, VisionConstants::VISION_LINEAR_D};
    frc::PIDController turnController{VisionConstants::VISION_ANGULAR_P, 0.0, VisionConstants::VISION_ANGULAR_D};
  
    double rotationSpeed;
    double forwardSpeed;

    int aprilTagID;

    std::optional<frc::DriverStation::Alliance> alliance;
};
