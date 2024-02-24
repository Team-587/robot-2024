// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
//#include <PhotonVersion.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonUtils.h>
#include <chrono>

#include "Constants.h"
#include "VisionSubsystem.h"

class NoteVisionSubsystem : public VisionSubsystem {
 public:
  NoteVisionSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  photon::PhotonCamera* GetCamera() override { return &camera; }

  units::meter_t GetCameraHeight() override { return VisionConstants::CAMERA_HEIGHT_NOTE; };
  units::meter_t GetTargetHeight() override { return VisionConstants::TARGET_HEIGHT_NOTE; };
  units::radian_t GetCameraPitch() override { return VisionConstants::CAMERA_PITCH_NOTE; };

  uint64_t GetMaxTargetLatency() override { return VisionConstants::MAX_TARGET_LATENCY; };

 private:
  
    photon::PhotonCamera camera{VisionConstants::cameraOne};

};
