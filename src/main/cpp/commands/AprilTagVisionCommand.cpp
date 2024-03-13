// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagVisionCommand.h"

AprilTagVisionCommand::AprilTagVisionCommand(
  AprilTagVisionSubsystem* pAprilTagVisionSubsystem, 
  DriveSubsystem* pDriveSubsystem,
  ShooterIntake* pShooterIntake,
  RobotArm* pRobotArm,
  ShootCommand* pShootCommand) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pAprilTagVisionSubsystem = pAprilTagVisionSubsystem;
  m_pDriveSubsystem = pDriveSubsystem;
  m_pShooterIntake = pShooterIntake;
  m_pRobotArm = pRobotArm;
  m_pShootCommand = pShootCommand;
  
  AddRequirements(m_pAprilTagVisionSubsystem);
  AddRequirements(m_pDriveSubsystem);

  turnController.SetTolerance(3, 5);
  turnController.SetIntegratorRange(-.05, .05);
  forwardController.SetTolerance(1, 3);
  forwardController.SetIntegratorRange(-.05, .05);
  
  int count = 0;
  m_DistanceBuckets[count++] = new DistanceBucket(1.57, 1.705, .6, 0, 0, 15);
  m_DistanceBuckets[count++] = new DistanceBucket(1.705, 1.955, .6, 0, 0, 18);
  m_DistanceBuckets[count++] = new DistanceBucket(1.955, 2.24, .7, 0, 0, 21);
  m_DistanceBuckets[count++] = new DistanceBucket(2.24, 2.59, .7, 0, 0, 24);
  m_DistanceBuckets[count++] = new DistanceBucket(2.59, 2.91, .6, 0, 0, 26);
  m_DistanceBuckets[count++] = new DistanceBucket(2.91, 3.2, .7, 0, 0, 28);
  m_DistanceBuckets[count++] = new DistanceBucket(3.2, 3.495, .7, 0, 0, 30);
  m_DistanceBuckets[count++] = new DistanceBucket(3.495, 3.785, .7, 0, 0, 32);
  m_DistanceBuckets[count++] = new DistanceBucket(3.785, 3.93, .75, 0, 0, 34);
}

// Called when the command is initially scheduled.
void AprilTagVisionCommand::Initialize() {
  alliance = frc::DriverStation::GetAlliance();
  std::cout << "AprilTagVisionCommand" << m_pAprilTagVisionSubsystem->GetCamera()->GetCameraTable().get()->GetPath() << "\n";
  frc::SmartDashboard::PutNumber("SV", 0);
  frc::SmartDashboard::PutNumber("AA", 45);
  frc::SmartDashboard::PutNumber("EH", 0);
  m_WaitCommand.Schedule();
}


std::optional<DistanceBucket*> AprilTagVisionCommand::GetDistanceBucket(double distance) { 

    for (int i = 0; i < 10; i++) {

      if (distance >= m_DistanceBuckets[i]->m_minDist && distance <= m_DistanceBuckets[i]->m_maxDist) {
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
    //frc::SmartDashboard::PutNumber("Distance", (double)distance.value());
    //std::cout << "Distance: " << (double)distance.value() << "\n";
    //double SV = frc::SmartDashboard::GetNumber("SV", 0);
    //double AA = frc::SmartDashboard::GetNumber("AA", 45);
    //double EH = frc::SmartDashboard::GetNumber("EH", 0);
    //m_pShooterIntake->setShooterVelocity(SV);
    //m_pRobotArm->ArmPosition(AA, EH);

    if (distance.has_value()) {
      std::optional<DistanceBucket*> distanceBucket = GetDistanceBucket((double)distance.value());
      if(distanceBucket.has_value()) {
        m_pShooterIntake->setShooterVelocity(distanceBucket.value()->m_shooterSpeed);
        m_pRobotArm->ArmPosition(distanceBucket.value()->m_armAngle, distanceBucket.value()->m_elevatorHeight);
      }
    }

    rotationSpeed = -turnController.Calculate(target.value().GetYaw(), 0);
    rotationSpeed = std::fmin(1, std::fmax(-1, rotationSpeed / 5)); 
    
    m_pDriveSubsystem->Drive(
        units::meters_per_second_t{m_driverController.GetLeftY()},
        units::meters_per_second_t{m_driverController.GetLeftX()},
        units::radians_per_second_t{rotationSpeed}, true);

    if(m_WaitCommand.IsFinished() && !m_pShootCommand->IsScheduled()){
      m_pShootCommand->Schedule();
    }
  }
}

// Called once the command ends or is interrupted.
void AprilTagVisionCommand::End(bool interrupted) {
  m_pShootCommand->Cancel();
}

// Returns true when the command should end.
bool AprilTagVisionCommand::IsFinished() {
  return false;
}
