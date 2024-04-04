// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include "Constants.h"
#include "AprilTagVisionSubsystem.h"
#include "NoteVisionSubsystem.h"
#include "ShooterIntake.h"
#include <commands/RumbleCommand.h>

class Lights : public frc2::SubsystemBase {
 public:
  Lights(
    AprilTagVisionSubsystem* pAprilTagVisionSubsystem,
    NoteVisionSubsystem* pNoteVisionSubsystem,
    ShooterIntake* pShooterIntake);



  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
AprilTagVisionSubsystem* m_pAprilTagVisionSubsystem;
NoteVisionSubsystem* m_pNoteVisionSubsystem;
ShooterIntake* m_pShooterIntake;
RumbleCommand m_rumbleCommand;

bool blueActive;
bool readyToRumble = false;
int ledLoopCount;
int disableLoopCount;
bool haveNoteLight;
frc::AddressableLED m_led{DriveConstants::kLEDPort};
std::array<frc::AddressableLED::LEDData, DriveConstants::kLEDsideLength> LEDsideArray;
std::array<frc::AddressableLED::LEDData, DriveConstants::kLEDtotalLength > topLEDArray;
std::array<frc::AddressableLED::LEDData, DriveConstants::kLEDbackLength > LEDbackArray;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
