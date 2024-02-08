// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TentaclesSubsystem.h"
#include "constants.h"

TentaclesSubsystem::TentaclesSubsystem() :

    #ifdef HAVETENTACLES
    LeftTentacleMotor(DriveConstants::kLeftTentacleMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    RightTentacleMotor(DriveConstants::kRightTentacleMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    #endif 
    TentacleExtend(false)
{
}

void TentaclesSubsystem::allowTentacleExtend() {
    TentacleExtend = true;
    
}

void TentaclesSubsystem::moveTentacles(double leftTentacleValue, double rightTentacleValue) {

    if(leftTentacleValue < 0.1 && leftTentacleValue > -0.1) {
        leftTentacleValue = 0;
    }

    if(rightTentacleValue < 0.1 && rightTentacleValue > -0.1) {
        rightTentacleValue = 0;
    } 

    #ifdef HAVETENTACLES
    if(TentacleExtend) {
        LeftTentacleMotor.Set(leftTentacleValue);
        RightTentacleMotor.Set(rightTentacleValue);
    }

    else {
        LeftTentacleMotor.Set(0);
        RightTentacleMotor.Set(0);
    }
    
    #endif
}

// This method will be called once per scheduler run
void TentaclesSubsystem::Periodic() {


}
