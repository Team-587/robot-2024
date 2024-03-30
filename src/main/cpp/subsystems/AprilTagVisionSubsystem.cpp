// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AprilTagVisionSubsystem.h"

/*AprilTagVisionSubsystem::AprilTagVisionSubsystem() {
    
};*/
AprilTagVisionSubsystem::AprilTagVisionSubsystem():
    VisionSubsystem() {
    alliance = frc::DriverStation::GetAlliance();
}

// This method will be called once per scheduler run
void AprilTagVisionSubsystem::Periodic() {

}

std::optional<photon::PhotonTrackedTarget> AprilTagVisionSubsystem::GetBestTarget() {
  //std::cout << "GetTarget: \n";
  const photon::PhotonPipelineResult result = GetCamera()->GetLatestResult();
  std::cout << "HasTarget: " << result.HasTargets() << "\n";
  if(result.HasTargets()) {

    const std::span<const photon::PhotonTrackedTarget> targets = result.GetTargets();
    for (const photon::PhotonTrackedTarget& target : targets) { 
        int id = target.GetFiducialId();
        if ((alliance.value() == frc::DriverStation::Alliance::kRed && id == VisionConstants::redAprilTag) || 
            (alliance.value() == frc::DriverStation::Alliance::kBlue && id == VisionConstants::blueAprilTag)) { 
            currentTarget = std::make_optional(target);
            currentTargetTime = GetTimeMillisec();
            std::cout << "Found Target" << "\n";
            return currentTarget;
        }
    }
  } else if (currentTarget != std::nullopt && GetTimeMillisec() - currentTargetTime  < GetMaxTargetLatency()) {
    std::cout << "Found Old Target" << "\n";
    return currentTarget;
  }
  std::cout << "No Target" << "\n";
  currentTarget = std::nullopt;
  return std::nullopt;
}
