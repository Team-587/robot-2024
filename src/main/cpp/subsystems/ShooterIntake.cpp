// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterIntake.h"
#include "Constants.h"

//Constructor for shooter intake
ShooterIntake::ShooterIntake():

    stateVar(STOP),
    intakeMotor(DriveConstants::kIntakeMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    outakeMotor(DriveConstants::kOutakeMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    motorSpeedPID(outakeMotor.GetPIDController()),
    intakeSwitch(DriveConstants::kIntakeSwitchPort),
    stopIntake(false),
    startIntake(false),
    shooterVelocity(0),
    delayCount(0)
{
    intakeMotor.SetSmartCurrentLimit(50);
    intakeMotor.SetSecondaryCurrentLimit(80);
    intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    intakeMotor.EnableVoltageCompensation(12);
    outakeMotor.SetSmartCurrentLimit(50);
    outakeMotor.SetSecondaryCurrentLimit(80);
    outakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    outakeMotor.EnableVoltageCompensation(12);
    intakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 100);
    intakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 50);
    intakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 50);
    outakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 100);
    outakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 50);
    outakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 50);
    intakeMotor.Set(0);
    outakeMotor.Set(0);
}

void ShooterIntake::setIntakeStart() {
    if(stateVar == STOP) {
        startIntake = true;
        stopIntake = false;
    } else {
        startIntake = false;
        stopIntake = true;
    }
}

void ShooterIntake::setIntakeStop() {
    if(stateVar == INTAKE) {
        stopIntake = true;
        startIntake = false;
    } else {
        stopIntake = false;
        startIntake = true;
    }
}

void ShooterIntake::setBeginShooter() {
 if(stateVar == SHOOTSETUP) {
    beginShooter = true;
   }  else {
    beginShooter = false;
   }
        

}

void ShooterIntake::setShooterVelocity(double velocity) {
    shooterVelocity = velocity; 
}

// This method will be called once per scheduler run
void ShooterIntake::Periodic() {
 bool switchState = intakeSwitch.Get();

  switch(stateVar) {
        case STOP:
            intakeMotor.Set(0);
            outakeMotor.Set(0);
                if(switchState == true) {
                    stateVar = HAVENOTE;
                } else if (startIntake == true) {
                    stateVar = INTAKE;
                    startIntake = false;
                }
            break;
        case INTAKE:
            intakeMotor.Set(0.5);
            outakeMotor.Set(0);
                if(switchState == true) {
                    stateVar = HAVENOTE;
                } else if (stopIntake == true) {
                    stateVar = STOP;
                    stopIntake = false;
                }
            break;
        case HAVENOTE:
            intakeMotor.Set(0);
            outakeMotor.Set(0);
                if(shooterVelocity > 0) {
                    stateVar = SHOOTSETUP;
                } 
            break;
        case SHOOTSETUP:
            outakeMotor.Set(shooterVelocity);
            intakeMotor.Set(0);
                if(beginShooter == true) {
                    stateVar = SHOOTING;
                    delayCount = 20;
                    beginShooter = false;
                } else if (shooterVelocity == 0) {
                    stateVar = HAVENOTE;
                }
            break;
        case SHOOTING:
            intakeMotor.Set(0.5);
            outakeMotor.Set(shooterVelocity);  
            delayCount--;
                if (delayCount <= 0) {
                    stateVar = STOP;
                }
            break;
/*        case REVERSE:
            
            break;
*/
  }

}
