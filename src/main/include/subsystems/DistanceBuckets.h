// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

#include "Constants.h"

class DistanceBuckets : public frc2::SubsystemBase {
 public:
    DistanceBuckets(units::meter_t minDist,
              units::meter_t maxDist,
              units::meter_t shooterSpeed,
              units::meter_t intakeSpeed,
              units::meter_t elevatorHeight,
              units::angle::radians armHeight);

    DistanceBuckets();

    units::meter_t m_minDist;
    units::meter_t m_maxDist;
    units::meter_t m_shooterSpeed;
    units::meter_t m_intakeSpeed;
    units::meter_t m_elevatorHeight;
    units::angle::radians m_armHeight;
  
  private:
    
};
