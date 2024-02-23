// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include "Constants.h"

class Lights : public frc2::SubsystemBase {
 public:
  Lights();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
bool blueActive;
int ledLoopCount;
int disableLoopCount;
bool haveNoteLight;
frc::AddressableLED m_led{DriveConstants::kLEDPort};
std::array<frc::AddressableLED::LEDData, DriveConstants::kLEDLength> ledArray;
std::array<frc::AddressableLED::LEDData, DriveConstants::kLEDLength * 3> topLEDArray;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
