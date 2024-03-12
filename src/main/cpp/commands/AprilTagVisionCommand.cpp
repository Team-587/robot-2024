// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagVisionCommand.h"

AprilTagVisionCommand::AprilTagVisionCommand(
  AprilTagVisionSubsystem* pAprilTagVisionSubsystem, 
  DriveSubsystem* pDriveSubsystem,
  ShooterIntake* pShooterIntake,
  RobotArm* pRobotArm) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pAprilTagVisionSubsystem = pAprilTagVisionSubsystem;
  m_pDriveSubsystem = pDriveSubsystem;
  m_pShooterIntake = pShooterIntake;
  m_pRobotArm = pRobotArm;
  
  AddRequirements(m_pAprilTagVisionSubsystem);
  AddRequirements(m_pDriveSubsystem);

  turnController.SetTolerance(3, 5);
  turnController.SetIntegratorRange(-.05, .05);
  forwardController.SetTolerance(1, 3);
  forwardController.SetIntegratorRange(-.05, .05);

  std::cout << "AprilTagVisionCommand" << m_pAprilTagVisionSubsystem->GetCamera()->GetCameraTable().get()->GetPath() << "\n";
  
}

// Called when the command is initially scheduled.
void AprilTagVisionCommand::Initialize() {
  alliance = frc::DriverStation::GetAlliance();
}


std::optional<DistanceBucket> AprilTagVisionCommand::GetDistanceBucket(double distance) { 

    for (int i = 0; i < 10; i++) {

      if (distance >= m_DistanceBuckets[i].m_minDist && distance <= m_DistanceBuckets[i].m_maxDist) {
        return std::make_optional(m_DistanceBuckets[i]);
      }
    }
    return std::nullopt;
  } 

// Called repeatedly when this Command is scheduled to run
void AprilTagVisionCommand::Execute() {

  //std::cout << "AprilTagVisionCommand\n";

  //const auto& result = m_pAprilTagVisionSubsystem->GetCamera()->GetLatestResult();

  const std::optional<photon::PhotonTrackedTarget> target = m_pAprilTagVisionSubsystem->GetBestTarget();

  frc::SmartDashboard::PutBoolean("AprilTargets", target.has_value());

  if (target.has_value()) {
    
    forwardSpeed = 0;
    aprilTagID = target.value().GetFiducialId();
    std::optional<units::meter_t> distance = m_pAprilTagVisionSubsystem->GetDistance();

    if (distance.has_value()) {
      std::optional<DistanceBucket> distanceBucket = GetDistanceBucket((double)distance.value());
      if(distanceBucket.has_value()) {
        m_pShooterIntake->setShooterVelocity(distanceBucket.value().m_shooterSpeed);
        m_pRobotArm->ArmPosition(distanceBucket.value().m_armAngle, distanceBucket.value().m_elevatorHeight);
      }
    }

    rotationSpeed = -turnController.Calculate(target.value().GetYaw(), 0);
    rotationSpeed = std::fmin(1, std::fmax(-1, rotationSpeed / 5)); 
    
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
