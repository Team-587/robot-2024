// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterIntake.h"
#include "Constants.h"
#include "iostream"
#include <frc/smartdashboard/Smartdashboard.h>

//Constructor for shooter intake
ShooterIntake::ShooterIntake():

    stateVar(STOP),
    #ifdef HAVEINTAKE
    intakeMotor(DriveConstants::kIntakeMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    outakeMotor(DriveConstants::kOutakeMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    motorSpeedPID(outakeMotor.GetPIDController()),
    #endif
    intakeSwitch(DriveConstants::kIntakeSwitchPort),
    startIntake(false),
    stopIntake(false),
    shooterVelocity(0),
    delayCount(0)
{
    #ifdef HAVEINTAKE
    intakeMotor.SetSmartCurrentLimit(50);
    intakeMotor.SetSecondaryCurrentLimit(80);
    intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    intakeMotor.EnableVoltageCompensation(12);
    outakeMotor.SetSmartCurrentLimit(50);
    outakeMotor.SetSecondaryCurrentLimit(80);
    outakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    outakeMotor.EnableVoltageCompensation(12);
    outakeMotor.SetInverted(true);
    intakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 100);
    intakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 50);
    intakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 50);
    outakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 100);
    outakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 50);
    outakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 50);
    intakeMotor.Set(0);
    outakeMotor.Set(0);
    frc::SmartDashboard::PutNumber("Intake Velocity Intake", 0.2);
    frc::SmartDashboard::PutNumber("Intake Velocity Shoot", 0.8);
    frc::SmartDashboard::PutNumber("Shooter Velocity Shoot", 0.4);
    #endif
}

void ShooterIntake::setIntakeStart() {
    std::cout<<"Intake Start\n";
    if(stateVar == STOP) {
        startIntake = true;
        stopIntake = false;
    } else {
        startIntake = false;
        stopIntake = true;
    }
}

void ShooterIntake::setIntakeStop() {
    std::cout<<"Intake Stop\n";
    if(stateVar == INTAKE) {
        stopIntake = true;
        startIntake = false;
    } else {
        stopIntake = false;
        startIntake = true;
    }
}

void ShooterIntake::setBeginShooter() {
 std::cout<<"Begin Shooter\n";
 if(stateVar == SHOOTSETUP) {
    beginShooter = true;
   }  else {
    beginShooter = false;
   }
        

}

void ShooterIntake::setShooterVelocity(double velocity) {
    shooterVelocity = velocity;
    std::cout<<shooterVelocity<<" Velocity\n";
}

// This method will be called once per scheduler run
void ShooterIntake::Periodic() {
 bool switchState =  !intakeSwitch.Get();
 double intakeVelocityIntake = frc::SmartDashboard::GetNumber("Intake Velocity Intake", 0.2);
 double intakeVelocityShoot = frc::SmartDashboard::GetNumber("Intake Velocity Shoot", 0.8);
 double shooterVelocityShoot = frc::SmartDashboard::GetNumber("Shooter Velocity Shoot", 0.4);

 //std::cout<<switchState<<" "<<stateVar<<" Switch State\n";

  switch(stateVar) {
        case STOP:
            #ifdef HAVEINTAKE
            intakeMotor.Set(0);
            outakeMotor.Set(0);
            #endif
                if(switchState == true) {
                    stateVar = HAVENOTE;
                    std::cout<<"Have Note\n";
                } else if (startIntake == true) {
                    stateVar = INTAKE;
                    startIntake = false;
                    std::cout<<"Start Intaking\n";
                }
            break;
        case INTAKE:
            #ifdef HAVEINTAKE
            intakeMotor.Set(intakeVelocityIntake);
            outakeMotor.Set(0);
            #endif
                if(switchState == true) {
                    stateVar = HAVENOTE;
                    std::cout<<"Have Note\n";
                } else if (stopIntake == true) {
                    stateVar = STOP;
                    stopIntake = false;
                    std::cout<<"Stop Intake\n";
                }
            break;
        case HAVENOTE:
            #ifdef HAVEINTAKE
            intakeMotor.Set(0);
            outakeMotor.Set(0);
            #endif
                if(shooterVelocity > 0) {
                    stateVar = SHOOTSETUP;
                    std::cout<<"Ready to shoot\n";
                } 
            break;
        case SHOOTSETUP:
            #ifdef HAVEINTAKE
            outakeMotor.Set(shooterVelocityShoot);
            intakeMotor.Set(0);

            #endif
                if(beginShooter == true) {
                    stateVar = SHOOTING;
                    delayCount = 80;
                    beginShooter = false;
                    std::cout<<"Shot Note\n";
                } else if (shooterVelocity == 0) {
                    stateVar = HAVENOTE;
                    std::cout<<"Not ready to shoot\n";
                }
            break;
        case SHOOTING:
            #ifdef HAVEINTAKE
            intakeMotor.Set(intakeVelocityShoot);
            outakeMotor.Set(shooterVelocityShoot);  
            #endif
            delayCount--;
                if (delayCount <= 0) {
                    stateVar = STOP;
                    std::cout<<"Done Shooting\n";
                }
            break;
/*        case REVERSE:
            
            break;
*/
  }

}
