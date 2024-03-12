// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

#include "Constants.h"

class DistanceBucket : public frc2::SubsystemBase {
 public:
    DistanceBucket(double minDist,
              double maxDist,
              double shooterSpeed,
              double intakeSpeed,
              double elevatorHeight,
              double armAngle);

    DistanceBucket();

    double m_minDist;
    double m_maxDist;
    double m_shooterSpeed;
    double m_intakeSpeed;
    double m_elevatorHeight;
    double m_armAngle;
  
  private:
    
};
