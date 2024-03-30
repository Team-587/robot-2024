// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/RobotArm.h"
#include "Constants.h"
#include "iostream"
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotArm::RobotArm() :
    #ifdef HAVEARM
    ElevatorMotor(DriveConstants::kElevatorMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    ElbowAMotor(DriveConstants::kElbowAMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    ElbowBMotor(DriveConstants::kElbowBMotorPort, rev::CANSparkLowLevel::MotorType::kBrushless),
    ElevatorHeightPID(ElevatorMotor.GetPIDController()),
    ElbowAnglePID(ElbowAMotor.GetPIDController()),
    //ElevatorEncoder(ElevatorMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)),
    ElevatorEncoder(ElevatorMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle)),
    ElbowAEncoder(ElbowAMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle)),
    //ElevatorLimit(ElevatorMotor.GetReverseLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed)),
    #endif
    ElevatorHeight(0.6),
    ElbowAngle(0.2),
    CurElevatorHeight(0.0),
    CurElbowAngle(0.0)

{
    #ifdef HAVEARM
    //ElevatorEncoder.SetPositionConversionFactor((std::numbers::pi * 1.88) / 15.0);
    //ElevatorEncoder.SetVelocityConversionFactor((std::numbers::pi * 1.88) / 15.0 / 60.0);

    ElevatorEncoder.SetPositionConversionFactor(10.2);
    ElevatorEncoder.SetVelocityConversionFactor(10.2 / 60.0);
    //ElevatorEncoder.SetPosition(0);

    ElevatorHeightPID.SetFeedbackDevice(ElevatorEncoder);
    ElevatorHeightPID.SetPositionPIDWrappingEnabled(false);
    ElevatorHeightPID.SetPositionPIDWrappingMinInput(0.4);
    ElevatorHeightPID.SetPositionPIDWrappingMaxInput(11);
    ElevatorHeightPID.SetP(elevatorP);
    ElevatorHeightPID.SetI(elevatorI);
    ElevatorHeightPID.SetD(elevatorD);
    ElevatorHeightPID.SetFF(0);
    ElevatorHeightPID.SetOutputRange(-1.0, 1.0);
    ElevatorMotor.SetInverted(true);
    //ElevatorMotor.SetOpenLoopRampRate(0.5);
    ElevatorMotor.SetSmartCurrentLimit(60);
    ElevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ElevatorMax);
    ElevatorMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ElevatorMin);
    ElevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);



    ElbowAEncoder.SetPositionConversionFactor(360);
    ElbowAEncoder.SetVelocityConversionFactor((360)/60.0);
    //ElbowAEncoder.SetPosition(0);

    ElbowAnglePID.SetFeedbackDevice(ElbowAEncoder);
    ElbowAnglePID.SetPositionPIDWrappingEnabled(true);
    ElbowAnglePID.SetPositionPIDWrappingMinInput(0);
    ElbowAnglePID.SetPositionPIDWrappingMaxInput(360);
    ElbowAnglePID.SetP(elbowP);
    ElbowAnglePID.SetI(elbowI);
    ElbowAnglePID.SetD(elbowD);
    ElbowAnglePID.SetFF(0);
    ElbowAnglePID.SetOutputRange(-1.0, 1.0);


    ElbowAMotor.SetOpenLoopRampRate(0.5);
    ElbowAMotor.SetSmartCurrentLimit(60);
    ElbowAMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ElbowAngleMax);
    ElbowAMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ElbowAngleMin);
    ElbowAMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    ElbowAMotor.SetInverted(true);

    ElbowBMotor.Follow(ElbowAMotor, true);
    //ElbowBMotor.SetInverted(false);
    ElbowBMotor.SetOpenLoopRampRate(0.5);
    ElbowBMotor.SetSmartCurrentLimit(60);
    ElbowBMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ElbowAngleMax);
    ElbowBMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ElbowAngleMin);
    ElbowBMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    #endif

    elbowPID.SetP(elbowP);
    elbowPID.SetI(elbowI);
    elbowPID.SetD(elbowD);
    frc::SmartDashboard::PutData("elbowPID", &elbowPID);
    elevatorPID.SetP(elevatorP);
    elevatorPID.SetI(elevatorI);
    elevatorPID.SetD(elevatorD);
    frc::SmartDashboard::PutData("elevatorPID", &elevatorPID);
}


void RobotArm::ArmPosition(double angle, double height)
{
  if (height > ElevatorMax) height = ElevatorMax;
  if (height < ElevatorMin) height = ElevatorMin;
  ElevatorHeight = height;

  if (angle > ElbowAngleMax) angle = ElbowAngleMax;
  if (angle < ElbowAngleMin) angle = ElbowAngleMin;
  ElbowAngle = angle;
  
  std::cout<<ElevatorHeight<<" Reposition Arm\n";
  std::cout<<ElbowAngle<<" Change Arm Angle\n";
}

void RobotArm::GetArmPos(double &angle, double &height)
{
  angle = CurElbowAngle;
  height = CurElevatorHeight;
}

// This method will be called once per scheduler run
void RobotArm::Periodic() 
{

    /*frc::XboxController m_codriverController{OIConstants::kCoDriverControllerPort};
    if(m_codriverController.GetBackButton()){
      double coLeftY = m_codriverController.GetLeftY();
      if(coLeftY > -0.1 && coLeftY < 0.1){
        coLeftY = 0;
      }*/
      
      //ElbowAngle = ElbowAngle + coLeftY*4.0; 
      //if (ElbowAngle > ElbowAngleMax) ElbowAngle = ElbowAngleMax;
      //if (ElbowAngle < ElbowAngleMin) ElbowAngle = ElbowAngleMin;

      /*(ElevatorHeight = ElevatorHeight + coLeftY * 1.0; 
      if (ElevatorHeight > ElevatorHeight) ElevatorHeight = ElevatorMax;
      if (ElevatorHeight < ElevatorHeight) ElevatorHeight = ElevatorMin;*/
    //}  

  if (elevatorP != elevatorPID.GetP() || elevatorI != elevatorPID.GetI() || elevatorD != elevatorPID.GetD()) {
    elevatorP = elevatorPID.GetP();
    elevatorI = elevatorPID.GetI();
    elevatorD = elevatorPID.GetD();

    #ifdef HAVEARM
    ElevatorHeightPID.SetP(elevatorP);
    ElevatorHeightPID.SetI(elevatorI);
    ElevatorHeightPID.SetD(elevatorD);
    #endif
  }

  if (elbowP != elbowPID.GetP() || elbowI != elbowPID.GetI() || elbowD != elbowPID.GetD()) {
    elbowP = elbowPID.GetP();
    elbowI = elbowPID.GetI();
    elbowD = elbowPID.GetD();

    #ifdef HAVEARM
    ElbowAnglePID.SetP(elbowP);
    ElbowAnglePID.SetI(elbowI);
    ElbowAnglePID.SetD(elbowD);
    #endif
  }

    
    #ifdef HAVEARM
    double ElevatorEncoderValue = ElevatorEncoder.GetPosition();
    double ElbowAEncoderValue = ElbowAEncoder.GetPosition();

    CurElevatorHeight = ElevatorEncoderValue;
    CurElbowAngle = ElbowAEncoderValue;

    /*if(ElevatorLimit.Get()){
      //std::cout << "Elevator Limit hit\n";
      //ElevatorEncoder.SetPosition(0);
      ElevatorEncoderValue = 0;
    }*/

    frc::SmartDashboard::PutNumber("Cur Elbow Angle", ElbowAEncoderValue);
    frc::SmartDashboard::PutNumber("Set Elbow Angle", ElbowAngle);
    frc::SmartDashboard::PutNumber("Cur Elev Height", ElevatorEncoderValue);
    frc::SmartDashboard::PutNumber("Set Elev Height", ElevatorHeight);

    //std::cout << "The Elbow A Encoder Value is: " << ElbowAEncoderValue << "  The Elevator Encoder Value is: " << ElevatorEncoderValue << "\n";
    //std::cout << "The Elevator Height is currently: " << ElevatorHeight << "  The Elbow Angle is currently: " << ElbowAngle;

    bool CurElevatorLong = ElevatorEncoderValue >= ElevatorMaxsafe;
    bool CurElbowHigh = ElbowAEncoderValue >= ElbowAngleMaxsafe;
    bool DesElevatorLong = ElevatorHeight >= ElevatorMaxsafe;
    bool DesElbowHigh = ElbowAngle >= ElbowAngleMaxsafe;

    /*if(CurElevatorLong == true && DesElevatorLong == false && CurElbowHigh == false){
       ElevatorHeightPID.SetReference(ElevatorMaxsafe, rev::CANSparkMax::ControlType::kPosition);
       ElbowAnglePID.SetReference(ElbowAngle, rev::CANSparkMax::ControlType::kPosition);
    }else if(CurElevatorLong == false && CurElbowHigh == true && DesElbowHigh == false){
        ElbowAnglePID.SetReference(ElbowAngleMaxsafe, rev::CANSparkMax::ControlType::kPosition);
        ElevatorHeightPID.SetReference(ElevatorHeight, rev::CANSparkMax::ControlType::kPosition);
    }else{
        ElbowAnglePID.SetReference(ElbowAngle, rev::CANSparkMax::ControlType::kPosition);
        ElevatorHeightPID.SetReference(ElevatorHeight, rev::CANSparkMax::ControlType::kPosition);
    }*/
    ElbowAnglePID.SetReference(ElbowAngle, rev::CANSparkMax::ControlType::kPosition);
    ElevatorHeightPID.SetReference(ElevatorHeight, rev::CANSparkMax::ControlType::kPosition);
    #endif
    


}
