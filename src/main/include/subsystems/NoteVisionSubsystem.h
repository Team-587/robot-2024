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

class NoteVisionSubsystem : public frc2::SubsystemBase {
 public:
  NoteVisionSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  std::optional<photon::PhotonTrackedTarget> GetBestTarget();
  
  std::optional<units::meter_t> GetDistance();

  uint64_t GetTimeMillisec() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  }

  photon::PhotonCamera* GetCamera() {
    return &camera;
  }

 private:
  photon::PhotonCamera camera{VisionConstants::cameraOne};

  std::optional<photon::PhotonTrackedTarget> currentTarget;
  uint64_t currentTargetTime = 0;

};
