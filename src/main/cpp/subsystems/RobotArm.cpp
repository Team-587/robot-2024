// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RobotArm.h"
#include "Constants.h"
#include "iostream"

RobotArm::RobotArm() :
    #ifdef HAVEARM
    ElevatorMotor(DriveConstants::kElevatorMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    ElbowAMotor(DriveConstants::kElbowAMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    ElbowBMotor(DriveConstants::kElbowBMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    ElevatorHeightPID(ElevatorMotor.GetPIDController()),
    ElbowAnglePID(ElbowAMotor.GetPIDController()),
    ElevatorEncoder(ElevatorMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
    ElbowAEncoder(ElevatorMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
    #endif
    ElevatorHeight(0.0),
    ElbowAngle(0.0)

{
    #ifdef HAVEARM
    ElevatorEncoder.SetPositionConversionFactor((std::numbers::pi * 1.751)/12.0);
    ElevatorEncoder.SetVelocityConversionFactor((std::numbers::pi * 1.751)/12.0/60.0);
    ElevatorEncoder.SetPosition(0);

    ElevatorHeightPID.SetP(.054);
    ElevatorHeightPID.SetI(0);
    ElevatorHeightPID.SetD(0);
    ElevatorHeightPID.SetFF(0);
    ElevatorHeightPID.SetOutputRange(-0.6, 0.85);
    ElevatorMotor.SetOpenLoopRampRate(0.5);
    ElevatorMotor.SetSmartCurrentLimit(60);
    ElevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ElevatorMax);
    ElevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ElevatorMin);
    ElevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);



    ElbowAEncoder.SetPositionConversionFactor((std::numbers::pi * 1.751)/12.0);
    ElbowAEncoder.SetVelocityConversionFactor((std::numbers::pi * 1.751)/12.0/60.0);
    ElbowAEncoder.SetPosition(0);

    ElbowAnglePID.SetP(.054);
    ElbowAnglePID.SetI(0);
    ElbowAnglePID.SetD(0);
    ElbowAnglePID.SetFF(0);
    ElbowAnglePID.SetOutputRange(-0.6, 0.85);
    ElbowAMotor.SetOpenLoopRampRate(0.5);
    ElbowAMotor.SetSmartCurrentLimit(60);
    ElbowAMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ElbowAngleMax);
    ElbowAMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ElbowAngleMin);
    ElbowAMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    ElbowBMotor.Follow(ElbowAMotor, true);
    ElbowBMotor.SetOpenLoopRampRate(0.5);
    ElbowBMotor.SetSmartCurrentLimit(60);
    ElbowBMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ElbowAngleMax);
    ElbowBMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ElbowAngleMin);
    ElbowBMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    #endif
}


void RobotArm::ArmPosition(double angle, double height)
{
  ElevatorHeight = height;
  ElbowAngle = angle;
  
  std::cout<<ElevatorHeight<<" Reposition Arm\n";
  std::cout<<ElbowAngle<<" Change Arm Angle\n";
}

// This method will be called once per scheduler run
void RobotArm::Periodic() 
{
    #ifdef HAVEARM
    double ElevatorEncoderValue = ElevatorEncoder.GetPosition();
    double ElbowAEncoderValue = ElbowAEncoder.GetPosition();

    bool CurElevatorLong = ElevatorEncoderValue >= ElevatorMaxsafe;
    bool CurElbowHigh = ElbowAEncoderValue >= ElbowAngleMaxsafe;
    bool DesElevatorLong = ElevatorHeight >= ElevatorMaxsafe;
    bool DesElbowHigh = ElbowAngle >= ElbowAngleMaxsafe;

    if(CurElevatorLong == true && DesElevatorLong == false && CurElbowHigh == false){
       ElevatorHeightPID.SetReference(ElevatorMaxsafe, rev::CANSparkMax::ControlType::kPosition);
       ElbowAnglePID.SetReference(ElbowAngle, rev::CANSparkMax::ControlType::kPosition);
    }else if(CurElevatorLong == false && CurElbowHigh == true && DesElbowHigh == false){
        ElbowAnglePID.SetReference(ElbowAngleMaxsafe, rev::CANSparkMax::ControlType::kPosition);
        ElevatorHeightPID.SetReference(ElevatorHeight, rev::CANSparkMax::ControlType::kPosition);
    }else{
        ElbowAnglePID.SetReference(ElbowAngle, rev::CANSparkMax::ControlType::kPosition);
        ElevatorHeightPID.SetReference(ElevatorHeight, rev::CANSparkMax::ControlType::kPosition);
    }
    #endif
}
