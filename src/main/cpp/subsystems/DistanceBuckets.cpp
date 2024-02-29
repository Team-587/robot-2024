// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DistanceBuckets.h"

DistanceBuckets::DistanceBuckets(units::meter_t minDist,
                     units::meter_t maxDist,
                     units::meter_t shooterSpeed,
                     units::meter_t intakeSpeed,
                     units::meter_t elevatorHeight,
                     units::angle::radians armHeight) { 
    
    m_minDist = minDist;
    m_maxDist = maxDist;
    m_shooterSpeed = shooterSpeed;
    m_intakeSpeed = intakeSpeed;
    m_elevatorHeight = elevatorHeight;
    m_armHeight = armHeight;
}

DistanceBuckets::DistanceBuckets() {}
