// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterIntake.h"
#include "Constants.h"
#include "iostream"
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc/DriverStation.h>

//Constructor for shooter intake
ShooterIntake::ShooterIntake(RobotArm* arm):

    stateVar(STOP),
    #ifdef HAVEINTAKE
    intakeMotor(DriveConstants::kIntakeMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    outakeMotor(DriveConstants::kOutakeMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    groundIntakeMotor(DriveConstants::kGroundIntakeMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    motorSpeedPID(outakeMotor.GetPIDController()),
    #endif
    intakeSwitch(DriveConstants::kIntakeSwitchPort),
    startIntake(false),
    stopIntake(false),
    shooterVelocity(0),
    delayCount(0),
    p_robotarm(arm)
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
    groundIntakeMotor.SetSmartCurrentLimit(50);
    groundIntakeMotor.SetSecondaryCurrentLimit(80);
    groundIntakeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    groundIntakeMotor.EnableVoltageCompensation(12);
    groundIntakeMotor.SetInverted(false);
    intakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 100);
    intakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 50);
    intakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 50);
    outakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 100);
    outakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 50);
    outakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 50);
    groundIntakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 100);
    groundIntakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 50);
    groundIntakeMotor.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 50);
    intakeMotor.Set(0);
    outakeMotor.Set(0);
    groundIntakeMotor.Set(0);
    frc::SmartDashboard::PutNumber("Intake Velocity Intake", 0.4);
    frc::SmartDashboard::PutNumber("Intake Velocity Shoot", 0.4);
    frc::SmartDashboard::PutNumber("Shooter Velocity Shoot", 0.4);
    #endif
}

void ShooterIntake::setIntakeStart() {
    std::cout<<"Intake Start\n";
    if(frc::DriverStation::IsTeleop()) {
         beginShooter = false;
    } else {
        if(stateVar == STOP) {
            startIntake = true;
            stopIntake = false;
        } else if(stateVar == INTAKE) {
            startIntake = true;
            stopIntake = false;
        }
    }
}

void ShooterIntake::setIntakeStop() {
    std::cout<<"Intake Stop\n";
    if(frc::DriverStation::IsTeleop()){
        beginShooter = false;
    } else {
        if(stateVar == INTAKE) {
            stopIntake = true;
            startIntake = false;
        } /*else {
            stopIntake = false;
            startIntake = true;
        }*/
    }
}

void ShooterIntake::setBeginShooter() {
 std::cout<<"Begin Shooter\n";
 if(frc::DriverStation::IsTeleop()) {
    beginShooter = true;
 } else {
    if(stateVar == SHOOTSETUP) {
        beginShooter = true;
    } else {
        beginShooter = false;
    }
 }    

}

void ShooterIntake::setShooterVelocity(double velocity) {
    //outakeMotor.Set(velocity);
    if(frc::DriverStation::IsTeleop()) {
        if (velocity > 0) {
            beginShooter = true;
        } else {
            beginShooter = false;
        }
    }
    shooterVelocity = velocity;
    std::cout<<shooterVelocity<<" Velocity\n";
}

void ShooterIntake::setIntakeVelocity(double velocity) {
    double ArmHeight;
    double ArmAngle;

    p_robotarm->GetArmPos(ArmAngle, ArmHeight);

    if(ArmAngle < RobotArm::PickUpAngle + 5.0 && ArmHeight < RobotArm::PickUpLength + 1.0) {

        if(velocity <= 0){
            groundIntakeMotor.Set(0);
        }else{
            groundIntakeMotor.Set(groundIntakeVelocity);
        }

    } else {
        groundIntakeMotor.Set(0);
    }

    intakeMotor.Set(velocity);
}

// This method will be called once per scheduler run
void ShooterIntake::Periodic() {
 bool switchState =  getIntakeSensorState();

 if(frc::DriverStation::IsTeleop()) {
    if (beginShooter) {
        ///std::cout << "set outtake motor\n";
        outakeMotor.Set(shooterVelocity);
    } else {
        //std::cout << "stop outake motor\n";
        outakeMotor.Set(0);
    }
 } else {
    //double intakeVelocityIntake = frc::SmartDashboard::GetNumber("Intake Velocity Intake", 0.2);
    //double intakeVelocityShoot = frc::SmartDashboard::GetNumber("Intake Velocity Shoot", 0.8);
    //double shooterVelocityShoot = frc::SmartDashboard::GetNumber("Shooter Velocity Shoot", 0.4);

    //std::cout<<switchState<<" "<<stateVar<<" Switch State\n";
    
    switch(stateVar) {
            case STOP:
                #ifdef HAVEINTAKE
                intakeMotor.Set(0);
                groundIntakeMotor.Set(0);
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

                double ArmHeight;
                double ArmAngle;

                p_robotarm->GetArmPos(ArmAngle, ArmHeight);
                
                if (ArmAngle < RobotArm::PickUpAngle + 5.0 && ArmHeight < RobotArm::PickUpLength + 1.0) {
                    intakeMotor.Set(intakeVelocity);
                    groundIntakeMotor.Set(groundIntakeVelocity);
                    outakeMotor.Set(0);
                } else {
                    intakeMotor.Set(0);
                    groundIntakeMotor.Set(0);
                    outakeMotor.Set(0);
                }

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
                groundIntakeMotor.Set(0);
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
                groundIntakeMotor.Set(0);
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
                intakeMotor.Set(intakeShootVelocity);
                groundIntakeMotor.Set(0);
                outakeMotor.Set(shooterVelocity);  
                #endif

                delayCount--;

                    if (delayCount <= 0) {
                        stateVar = STOP;
                        
                        std::cout<<"Done Shooting\n";
                        shooterVelocity = 0;

                    }
                break;
    /*        case REVERSE:
                
                break;
    */
    }
 }
}
