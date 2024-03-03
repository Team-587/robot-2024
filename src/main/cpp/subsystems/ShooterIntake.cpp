// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterIntake.h"
#include "Constants.h"
#include "iostream"
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/DriverStation.h>

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
    intakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
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
    frc::SmartDashboard::PutNumber("Intake Velocity Intake", 0.4);
    frc::SmartDashboard::PutNumber("Intake Velocity Shoot", 0.4);
    frc::SmartDashboard::PutNumber("Shooter Velocity Shoot", 0.4);
    #endif
}

void ShooterIntake::setIntakeStart() {
    std::cout<<"Intake Start\n";
    startIntake = true;
    startIntakeShoot = false;
    beginShooter = false;
    /*bool switchState =  getIntakeSensorState();

    if (switchState == false) {
        stateVar = STOP;
    }

    if(stateVar == STOP) {
        startIntake = true;
        stopIntake = false;
    } else if(stateVar == INTAKE) {
        startIntake = true;
        stopIntake = false;
    }*/
}

void ShooterIntake::setIntakeStop() {
    std::cout<<"Intake Stop\n";
    startIntake = false;
    startIntakeShoot = false;
    beginShooter = false;
    /*if(stateVar == INTAKE) {
        stopIntake = true;
        startIntake = false;
    }*/ /*else {
        stopIntake = false;
        startIntake = true;
    }*/
}

void ShooterIntake::setBeginShooter() {
 std::cout<<"Begin Shooter\n";
 beginShooter = true;
 startIntakeShoot = false;
 startIntake = true;
 /*if(stateVar == SHOOTSETUP) {
    beginShooter = true;
   }*/  /*else {
    beginShooter = false;
   }*/
        

}

void ShooterIntake::setIntakeShoot() {
    startIntake = false;
    startIntakeShoot = true;
    beginShooter = true;
    //std::cout << "switchState: " << getIntakeSensorState() << "\n";
    //std::cout << "shooterVelocity: " << shooterVelocity << "\n";
    //std::cout << "intakeVelocity: " << intakeVelocity << "\n";
}

void ShooterIntake::setShooterVelocity(double velocity) {
    if (velocity > 0) {
        beginShooter = true;
        startIntake = false;
        startIntakeShoot = false;
    } else {
        beginShooter = false;
        startIntake = false;
        startIntakeShoot = false;
    }
    shooterVelocity = velocity;
    std::cout<<shooterVelocity<<" Velocity\n";
}

// This method will be called once per scheduler run
void ShooterIntake::Periodic() {
 bool switchState =  getIntakeSensorState();
 
 
 if (startIntake && !switchState) {
    intakeMotor.Set(intakeVelocity);
    //std::cout << "set intake motor\n";
    //std::cout << "switchState: " << getIntakeSensorState() << "\n";
    //std::cout << "shooterVelocity: " << shooterVelocity << "\n";
    //std::cout << "intakeVelocity: " << intakeVelocity << "\n";

 } else if (startIntakeShoot) {
    intakeMotor.Set(intakeShootVelocity);
 } else {
    //std::cout << "stop intake motor\n";
    intakeMotor.Set(0);
 }

 if (beginShooter) {
    ///std::cout << "set outtake motor\n";
    outakeMotor.Set(shooterVelocity);
 } else {
    //std::cout << "stop outake motor\n";
    outakeMotor.Set(0);
 }
 //double intakeVelocityIntake = frc::SmartDashboard::GetNumber("Intake Velocity Intake", 0.2);
 //double intakeVelocityShoot = frc::SmartDashboard::GetNumber("Intake Velocity Shoot", 0.8);
 //double shooterVelocityShoot = frc::SmartDashboard::GetNumber("Shooter Velocity Shoot", 0.4);

 //std::cout<<switchState<<" "<<stateVar<<" Switch State\n";
/*
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
            intakeMotor.Set(intakeVelocity);
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
            outakeMotor.Set(shooterVelocity);
            intakeMotor.Set(0);
            #endif

                if(beginShooter == true) {
                    stateVar = SHOOTING;
                    delayCount = 10;
                    beginShooter = false;

                    std::cout<<"Shot Note\n";

                } else if (shooterVelocity == 0) {
                    stateVar = HAVENOTE;

                    std::cout<<"Not ready to shoot\n";

                }
            break;

        case SHOOTING:
            #ifdef HAVEINTAKE
            intakeMotor.Set(intakeVelocity);
            outakeMotor.Set(shooterVelocity);  
            #endif

            delayCount--;

                if (delayCount <= 0) {
                    stateVar = STOP;
                    
                    std::cout<<"Done Shooting\n";
                    //if(frc::DriverStation::IsAutonomousEnabled()){
                        shooterVelocity = 0;
                    //}

                }
            break;

  }
*/
}
