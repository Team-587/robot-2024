// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonUtils.h>
#include <chrono>

class VisionSubsystem : public frc2::SubsystemBase {
 public:
  VisionSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  virtual photon::PhotonCamera* GetCamera() { return NULL; };

  virtual units::meter_t GetCameraHeight() { return units::meter_t(0); };
  virtual units::meter_t GetTargetHeight() { return units::meter_t(0); };
  virtual units::radian_t GetCameraPitch() { return units::radian_t(0); };
  virtual uint64_t GetMaxTargetLatency() { return 0; };

  std::optional<photon::PhotonTrackedTarget> GetBestTarget();
  
  std::optional<units::meter_t> GetDistance();

  uint64_t GetTimeMillisec() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
  }

 protected:
  
  std::optional<photon::PhotonTrackedTarget> currentTarget;
  uint64_t currentTargetTime = 0;

};
