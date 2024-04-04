// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

VisionSubsystem::VisionSubsystem() = default;

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {}

std::optional<photon::PhotonTrackedTarget> VisionSubsystem::GetBestTarget() {
  const photon::PhotonPipelineResult& result = GetCamera()->GetLatestResult();
  if(result.HasTargets()) {
    currentTarget = std::make_optional(result.GetBestTarget());
    currentTargetTime = GetTimeMillisec();
    return currentTarget;
  } else if (currentTarget != std::nullopt && GetTimeMillisec() - currentTargetTime < GetMaxTargetLatency()) {
    return currentTarget;
  } else {
    currentTarget = std::nullopt;
    return std::nullopt;
  }
}

bool VisionSubsystem::HasTargets() 
{
    return currentTarget != std::nullopt && GetTimeMillisec() - currentTargetTime < GetMaxTargetLatency();
}

std::optional<units::meter_t> VisionSubsystem::GetDistance(photon::PhotonTrackedTarget *pTarget) {
  if(!pTarget) {
    return std::nullopt;
  }
  units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
      GetCameraHeight(), 
      GetTargetHeight(), 
      GetCameraPitch(),
      units::degree_t{pTarget->GetPitch()});
  return std::make_optional(range);

   
}