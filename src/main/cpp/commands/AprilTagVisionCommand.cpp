// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagVisionCommand.h"
#include "RobotContainer.h"

AprilTagVisionCommand::AprilTagVisionCommand(
  AprilTagVisionSubsystem* pAprilTagVisionSubsystem, 
  DriveSubsystem* pDriveSubsystem,
  ShooterIntake* pShooterIntake,
  RobotArm* pRobotArm) {
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


std::optional<DistanceBuckets> AprilTagVisionCommand::GetDistances(units::meter_t distance) { 

    for (int i = 0; i < 10; i++) {

      if (distance >= distArray[i].m_minDist && distance <= distArray[i].m_maxDist) {
        return std::make_optional(distArray[i]);

      } else {
        return std::nullopt;

      }
    }
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
      std::optional<DistanceBuckets> distanceBuckets = GetDistances(distance.value());

      if(distanceBuckets.has_value()) {
        m_pShooterIntake->setShooterVelocity((double)distanceBuckets.value().m_shooterSpeed);
        //distanceBuckets.value().m_intakeSpeed;
        distanceBuckets.value().m_maxDist;
        distanceBuckets.value().m_minDist;
        m_pRobotArm->ElevatorHeight = (double)distanceBuckets.value().m_elevatorHeight;
        m_pRobotArm->ElbowAngle = (double)distanceBuckets.value().m_armHeight;
      }

    }

    rotationSpeed = m_driverController.GetRightX();
    if ((alliance.value() == frc::DriverStation::Alliance::kRed && aprilTagID == VisionConstants::redAprilTag) || 
        (alliance.value() == frc::DriverStation::Alliance::kBlue && aprilTagID == VisionConstants::blueAprilTag)) { 
        rotationSpeed = -turnController.Calculate(target.value().GetYaw(), 0);
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
