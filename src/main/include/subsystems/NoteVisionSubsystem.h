// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
//#include <PhotonVersion.h>
#include <photon/PhotonCamera.h>

#include "Constants.h"

class NoteVisionSubsystem : public frc2::SubsystemBase {
 public:
  NoteVisionSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  photon::PhotonCamera* GetCamera() {
    return &camera;
  }

 private:
    photon::PhotonCamera camera{VisionConstants::cameraOne};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
