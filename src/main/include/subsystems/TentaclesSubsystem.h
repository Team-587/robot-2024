// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#define HAVETENTACLES

class TentaclesSubsystem : public frc2::SubsystemBase {
 public:

  TentaclesSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  void allowTentacleExtend();

  void moveTentacles(double leftTentacleValue, double rightTentacleValue);

  void Periodic() override;

 private:

 bool TentacleExtend;

  #ifdef HAVETENTACLES
  rev::CANSparkMax RightTentacleMotor;
  rev::CANSparkMax LeftTentacleMotor;
  #endif
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
