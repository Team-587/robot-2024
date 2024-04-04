// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DistanceBucket.h"

DistanceBucket::DistanceBucket(double distance,
                    double shooterSpeed,
                    double armAngle) { 
  m_distance = distance; 
  m_shooterSpeed = shooterSpeed;
  m_armAngle = armAngle;
}

DistanceBucket::DistanceBucket(double distance,
                    double minDist,
                    double maxDist,
                    double shooterSpeed,
                    double armAngle) { 
    m_distance = distance;
    m_minDist = minDist;
    m_maxDist = maxDist;
    m_shooterSpeed = shooterSpeed;
    m_armAngle = armAngle;
}

DistanceBucket::DistanceBucket() {}
