// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/NoteVisionSubsystem.h"

NoteVisionSubsystem::NoteVisionSubsystem() = default;

// This method will be called once per scheduler run
void NoteVisionSubsystem::Periodic() {
    
}

std::optional<photon::PhotonTrackedTarget> NoteVisionSubsystem::GetBestTarget() {
  const auto& result = camera.GetLatestResult();
  if(result.HasTargets()) {
    currentTarget = std::make_optional(result.GetBestTarget());
    currentTargetTime = GetTimeMillisec();
    return currentTarget;
  } else if (currentTargetTime - GetTimeMillisec() < VisionConstants::MAX_TARGET_LATENCY) {
    return currentTarget;
  } else {
    currentTarget = std::nullopt;
    return std::nullopt;
  }
}

std::optional<units::meter_t> NoteVisionSubsystem::GetDistance() {
  if(!currentTarget.has_value()) {
    return std::nullopt;
  }
  units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
      VisionConstants::CAMERA_HEIGHT_NOTE, 
      VisionConstants::TARGET_HEIGHT_NOTE, 
      VisionConstants::CAMERA_PITCH_NOTE,
      units::degree_t{currentTarget.value().GetPitch()});
  return std::make_optional(range);
}
