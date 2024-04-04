// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AprilTagVisionCommand.h"

AprilTagVisionCommand::AprilTagVisionCommand(
  AprilTagVisionSubsystem* pAprilTagVisionSubsystem, 
  DriveSubsystem* pDriveSubsystem,
  ShooterIntake* pShooterIntake,
  RobotArm* pRobotArm,
  ShootCommand* pShootCommand) :
    m_VisionThread{"AprilTag", pAprilTagVisionSubsystem},
    m_rumbleCommand{
  frc::GenericHID::RumbleType::kRightRumble,
  1_s, 1_s, 1
  }  {
  // Use addRequirements() here to declare subsystem dependencies.
  m_pAprilTagVisionSubsystem = pAprilTagVisionSubsystem;
  m_pDriveSubsystem = pDriveSubsystem;
  m_pShooterIntake = pShooterIntake;
  m_pRobotArm = pRobotArm;
  m_pShootCommand = pShootCommand;
  
  AddRequirements(m_pAprilTagVisionSubsystem);
  AddRequirements(m_pShooterIntake);
  AddRequirements(m_pDriveSubsystem);

  //turnController.SetTolerance(3, 5);
  //turnController.SetIntegratorRange(-.05, .05);
  //forwardController.SetTolerance(1, 3);
  //forwardController.SetIntegratorRange(-.05, .05);
  
  int count = 0;
  m_DistanceBuckets[count++] = new DistanceBucket(1.48, 3700, 13);
  m_DistanceBuckets[count++] = new DistanceBucket(1.6, 3700, 14);
  m_DistanceBuckets[count++] = new DistanceBucket(1.7, 4000, 14);
  m_DistanceBuckets[count++] = new DistanceBucket(1.8, 4000, 15);
  m_DistanceBuckets[count++] = new DistanceBucket(1.9, 4000, 16.5);
  m_DistanceBuckets[count++] = new DistanceBucket(2, 4200, 19);
  m_DistanceBuckets[count++] = new DistanceBucket(2.1, 4200, 20);
  m_DistanceBuckets[count++] = new DistanceBucket(2.2, 4300, 20.5);
  m_DistanceBuckets[count++] = new DistanceBucket(2.3, 4300, 20.8);
  m_DistanceBuckets[count++] = new DistanceBucket(2.4, 4400, 22.7);
  m_DistanceBuckets[count++] = new DistanceBucket(2.5, 4400, 22.7);
  m_DistanceBuckets[count++] = new DistanceBucket(2.6, 4400, 23);
  m_DistanceBuckets[count++] = new DistanceBucket(2.7, 4500, 24.5);
  m_DistanceBuckets[count++] = new DistanceBucket(2.8, 4800, 24.5);
  m_DistanceBuckets[count++] = new DistanceBucket(2.9, 5000, 24.5);
  m_DistanceBuckets[count++] = new DistanceBucket(3, 5200, 24.5);
  m_DistanceBuckets[count++] = new DistanceBucket(3.1, 5300, 24.5);
  m_DistanceBuckets[count++] = new DistanceBucket(3.2, 5400, 24.5);
  m_DistanceBuckets[count++] = new DistanceBucket(3.3, 5500, 24.7);
  m_DistanceBuckets[count++] = new DistanceBucket(3.4, 5600, 25);
  m_DistanceBuckets[count++] = new DistanceBucket(3.5, 5800, 26.5);
  m_DistanceBuckets[count++] = new DistanceBucket(3.6, 6000, 26.5);
  m_DistanceBuckets[count++] = new DistanceBucket(3.7, 6100, 27);

  frc::SmartDashboard::PutNumber("SV", 4200);
  frc::SmartDashboard::PutNumber("AA", 25);
  //frc::SmartDashboard::PutNumber("EH", 0);
  frc::SmartDashboard::PutNumber("EnableTest", 0);
  m_VisionThread.Start();
}

// Called when the command is initially scheduled.
void AprilTagVisionCommand::Initialize() {
  //alliance = frc::DriverStation::GetAlliance();
  std::cout << "AprilTagVisionCommand" << m_pAprilTagVisionSubsystem->GetCamera()->GetCameraTable().get()->GetPath() << "\n";
  m_WaitCommand.Schedule();
  m_VisionThread.Enable(true);
}


std::optional<DistanceBucket*> AprilTagVisionCommand::GetDistanceBucket(double distance) { 
  DistanceBucket *pPrevious;
  for (int i = 0; i < BUCKET_COUNT; i++) {
    if (i > 0 && distance >= pPrevious->m_distance && distance <= m_DistanceBuckets[i]->m_distance) {
      double totalDistance = m_DistanceBuckets[i]->m_distance - pPrevious->m_distance;
      double ratio = (distance - pPrevious->m_distance) / totalDistance;
      double shooter = pPrevious->m_shooterSpeed + (m_DistanceBuckets[i]->m_shooterSpeed - pPrevious->m_shooterSpeed) * ratio;
      double arm = pPrevious->m_armAngle + (m_DistanceBuckets[i]->m_armAngle - pPrevious->m_armAngle) * ratio;
      DistanceBucket *pDistanceBucket = new DistanceBucket(distance, pPrevious->m_distance, m_DistanceBuckets[i]->m_distance, shooter, arm);
      return std::make_optional(pDistanceBucket);
    }
    pPrevious = m_DistanceBuckets[i];
  }
  return std::nullopt;
} 

// Called repeatedly when this Command is scheduled to run
void AprilTagVisionCommand::Execute() {

  double rotationSpeed = m_driverController.GetRightX();
  photon::PhotonTrackedTarget *pTarget = m_VisionThread.GetTarget();
  //std::cout << "AprilTagVisionCommand1\n";
  if(pTarget) {
    //std::cout << "AprilTagVisionCommand2\n";
    std::optional<units::meter_t> distance = m_pAprilTagVisionSubsystem->GetDistance(pTarget);
    if (distance.has_value()) {
      //std::cout << "AprilTagVisionCommand3\n";
      frc::SmartDashboard::PutNumber("Distance", (double)distance.value());
      double ET = frc::SmartDashboard::GetNumber("EnableTest", 0);
      if(ET > 0) {
        double SV = frc::SmartDashboard::GetNumber("SV", 4200);
        double AA = frc::SmartDashboard::GetNumber("AA", 25);
        //double EH = frc::SmartDashboard::GetNumber("EH", 0);
        std::cout << "AprilTagVisionCommand - Shoot: " << SV << ", Arm: " << AA << "\n";
        m_pShooterIntake->setShooterVelocity(SV);
        m_pRobotArm->ArmPosition(AA, 0);
        if(m_WaitCommand.IsFinished()){
          m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::intakeShootVelocity);
        }
      } else {
        //std::cout << "AprilTagVisionCommand4\n";
        std::optional<DistanceBucket*> distanceBucket = GetDistanceBucket((double)distance.value());
        if(distanceBucket.has_value()) {
          //std::cout << "AprilTagVisionCommand5\n";
          if(m_rumbleCommand.IsFinished()){
            m_rumbleCommand.IsScheduled();
          }
          std::cout << "AprilTagVisionCommand - distance: " << (double)distance.value() << ", min: " << distanceBucket.value()->m_minDist << ", max: " << distanceBucket.value()->m_maxDist << ", Shoot: " << distanceBucket.value()->m_shooterSpeed << ", Arm: " << distanceBucket.value()->m_armAngle << "\n";
          m_pShooterIntake->setShooterVelocity(distanceBucket.value()->m_shooterSpeed);
          m_pRobotArm->ArmPosition(distanceBucket.value()->m_armAngle, 0);
          delete distanceBucket.value();
          if(m_WaitCommand.IsFinished()){
            m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::intakeShootVelocity);
          }
        }else{
          m_rumbleCommand.Cancel();
        }
      }
      if (false && fabs(pTarget->GetYaw()) > 3) {
        rotationSpeed = pTarget->GetYaw() > 0 ? .1 : -.1;
        std::cout << "AprilTagVisionCommand - Yaw: " << pTarget->GetYaw() << ", Rotation: " << rotationSpeed << "\n";
      } else {
        rotationSpeed = 0;
      }
    }else{
      m_rumbleCommand.Cancel();
    }
  }else{
    m_rumbleCommand.Cancel();
  }
  m_pDriveSubsystem->Drive(
        units::meters_per_second_t{m_driverController.GetLeftY()},
        units::meters_per_second_t{m_driverController.GetLeftX()},
        units::radians_per_second_t{rotationSpeed}, true);

/*
  //std::cout << "AprilTagVisionCommand\n";
  const std::optional<photon::PhotonTrackedTarget> target = m_pAprilTagVisionSubsystem->GetBestTarget();
  //std::cout << "AprilTagVisionCommand1\n";
  frc::SmartDashboard::PutBoolean("AprilTargets", target.has_value());
  //std::cout << "AprilTagVisionCommand2\n";
  if (target.has_value()) {
    //std::cout << "AprilTagVisionCommand3\n";
    //std::cout << "AprilTagVisionCommand4\n";
    aprilTagID = target.value().GetFiducialId();
    //std::cout << "AprilTagVisionCommand5\n";
    std::optional<units::meter_t> distance = m_pAprilTagVisionSubsystem->GetDistance();
    //std::cout << "AprilTagVisionCommand6\n";
    
    if (distance.has_value()) {
      frc::SmartDashboard::PutNumber("Distance", (double)distance.value());
      //std::cout << "AprilTagVisionCommand7\n";
      //std::cout << "Distance: " << (double)distance.value() << "\n";
      //std::cout << "AprilTagVisionCommand8\n";
      double ET = frc::SmartDashboard::GetNumber("EnableTest", 1);
      if(ET > 0) {
        double SV = frc::SmartDashboard::GetNumber("SV", 4200);
        double AA = frc::SmartDashboard::GetNumber("AA", 25);
        double EH = frc::SmartDashboard::GetNumber("EH", 0);
        std::cout << "AprilTagVisionCommand - Shoot: " << SV << ", Arm: " << AA << ", Elevator: " << EH <<"\n";
        m_pShooterIntake->setShooterVelocity(SV);
        m_pRobotArm->ArmPosition(AA, EH);
        //std::cout << "AprilTagVisionCommand - check wait\n";
        if(m_WaitCommand.IsFinished()){// && !m_pShootCommand->IsScheduled()){
          //std::cout << "AprilTagVisionCommand - Shoot Test\n";
          //m_pShootCommand->Schedule();
          m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::intakeShootVelocity);
        }
      } else {
        std::optional<DistanceBucket*> distanceBucket = GetDistanceBucket((double)distance.value());
        //std::cout << "AprilTagVisionCommand9\n";
        if(distanceBucket.has_value()) {
          //std::cout << "AprilTagVisionCommand10\n";
          std::cout << "AprilTagVisionCommand shooter speed: " << distanceBucket.value()->m_shooterSpeed << "\n";
          //m_pShooterIntake->setShooterVelocity(distanceBucket.value()->m_shooterSpeed);
          //std::cout << "AprilTagVisionCommand11\n";
          //m_pRobotArm->ArmPosition(distanceBucket.value()->m_armAngle, distanceBucket.value()->m_elevatorHeight);
          //std::cout << "AprilTagVisionCommand12\n";

          if(m_WaitCommand.IsFinished()){// && !m_pShootCommand->IsScheduled()){
            //std::cout << "AprilTagVisionCommand14\n";
            //m_pShootCommand->Schedule();
            m_pShooterIntake->setIntakeVelocity(ShooterIntakeConstants::intakeShootVelocity);
          }
        }
      }
      if (fabs(target.value().GetYaw()) > 1) {
        rotationSpeed = target.value().GetYaw() > 0 ? .2 : -.2;
        std::cout << "AprilTagVisionCommand - Yaw: " << target.value().GetYaw() << ", Rotation: " << rotationSpeed << "\n";
      }
    }

    //rotationSpeed = -turnController.Calculate(target.value().GetYaw(), 0);
    //rotationSpeed = std::fmin(1, std::fmax(-1, rotationSpeed / 5)); 
    
    
    //rotationSpeed = target.value().GetYaw() ? -.1 : .1;
    
    //std::cout << "AprilTagVisionCommand: " << rotationSpeed << "\n";
    //std::cout << "AprilTagVisionCommand13\n";
    
    //std::cout << "AprilTagVisionCommand15\n";
  } //else {
    //rotationSpeed = m_driverController.GetRightX();
  //}
  m_pDriveSubsystem->Drive(
        units::meters_per_second_t{m_driverController.GetLeftY()},
        units::meters_per_second_t{m_driverController.GetLeftX()},
        units::radians_per_second_t{rotationSpeed}, true);
*/
}

// Called once the command ends or is interrupted.
void AprilTagVisionCommand::End(bool interrupted) {
  //if(m_pShootCommand->IsScheduled())
  //  m_pShootCommand->Cancel();
  //std::cout << "AprilTagVisionCommand - End\n";
  m_pShooterIntake->setIntakeVelocity(0);
  m_VisionThread.Enable(false);
  
}

// Returns true when the command should end.
bool AprilTagVisionCommand::IsFinished() {
  return false;
}
