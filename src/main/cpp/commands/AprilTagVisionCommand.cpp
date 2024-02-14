// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagVisionCommand.h"

AprilTagVisionCommand::AprilTagVisionCommand(AprilTagVisionSubsystem* pAprilTagVisionSubsystem, DriveSubsystem* pDriveSubsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pAprilTagVisionSubsystem = pAprilTagVisionSubsystem;
  m_pDriveSubsystem = pDriveSubsystem;

  AddRequirements(m_pAprilTagVisionSubsystem);
  AddRequirements(m_pDriveSubsystem);

  turnController.SetTolerance(3, 5);
  turnController.SetIntegratorRange(-.05, .05);
  forwardController.SetTolerance(1, 3);
  forwardController.SetIntegratorRange(-.05, .05);
}

// Called when the command is initially scheduled.
void AprilTagVisionCommand::Initialize() {
  alliance = frc::DriverStation::GetAlliance();
}

// Called repeatedly when this Command is scheduled to run
void AprilTagVisionCommand::Execute() {

  const auto& result = m_pAprilTagVisionSubsystem->GetCamera()->GetLatestResult();

  frc::SmartDashboard::PutBoolean("AprilTargets", result.HasTargets());

  if (result.HasTargets()) {

    units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
        VisionConstants::CAMERA_HEIGHT, 
        VisionConstants::TARGET_HEIGHT, 
        VisionConstants::CAMERA_PITCH,
        units::degree_t{result.GetBestTarget().GetPitch()});
    
    forwardSpeed = 0;
    aprilTagID = result.GetBestTarget().GetFiducialId();

    rotationSpeed = m_driverController.GetRightX();
    if ((alliance.value() == frc::DriverStation::Alliance::kRed && aprilTagID == VisionConstants::redAprilTag) || 
        (alliance.value() == frc::DriverStation::Alliance::kBlue && aprilTagID == VisionConstants::blueAprilTag)) { 
        rotationSpeed = -turnController.Calculate(result.GetBestTarget().GetYaw(), 0);
        rotationSpeed = std::fmin(1, std::fmax(-1, rotationSpeed / 5));
    } 
    m_pDriveSubsystem->Drive(
        units::meters_per_second_t{m_driverController.GetLeftY()},
        units::meters_per_second_t{m_driverController.GetLeftX()},
        units::radians_per_second_t{rotationSpeed}, true);
  }
}

// Called once the command ends or is interrupted.
void AprilTagVisionCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AprilTagVisionCommand::IsFinished() {
  return false;
}
