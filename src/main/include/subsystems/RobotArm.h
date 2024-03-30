// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>

#define HAVEARM

class RobotArm : public frc2::SubsystemBase {
 public:
  RobotArm();

  void ArmPosition(double angle, double height);

  void GetArmPos(double &angle, double &height);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  static constexpr double PickUpAngle{0.2};
  static constexpr double PickUpLength{0.6};
  static constexpr double HoldAngle{0.2};
  static constexpr double HoldLength{0.6};
  static constexpr double AmpAngle{87.0};
  static constexpr double AmpLength{7.9};
  //static constexpr double ShortShootAngle{14.0};
  static constexpr double ShortShootAngle{3.0};
  static constexpr double ShortShootLength{0.6};
  static constexpr double LongShootAngle{19.0};
  static constexpr double LongShootLength{0.6};

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  #ifdef HAVEARM
  rev::CANSparkMax ElevatorMotor;
  rev::CANSparkMax ElbowAMotor;
  rev::CANSparkMax ElbowBMotor;
  rev::SparkPIDController ElevatorHeightPID;
  rev::SparkPIDController ElbowAnglePID;
  //rev::SparkRelativeEncoder ElevatorEncoder;
  rev::SparkAbsoluteEncoder ElevatorEncoder;
  rev::SparkAbsoluteEncoder ElbowAEncoder;

  //,k==================  SparkLimitSwitch ElevatorLimit;
  #endif

  const double ElevatorMin{0.4};
  const double ElevatorMax{8.3};
  const double ElevatorMaxsafe{5.0};
  const double ElbowAngleMin{0.0};
  const double ElbowAngleMax{91.0};
  const double ElbowAngleMaxsafe{14.0};

  frc::PIDController elbowPID{elbowP, elbowI, elbowD};
  double elbowP = 0.035;
  double elbowI = 0.0;
  double elbowD = 0;

  frc::PIDController elevatorPID{elevatorP, elevatorI, elevatorD};
  double elevatorP = 0.5;
  double elevatorI = 0.0;
  double elevatorD = 0.0;

  double ElevatorHeight;
  double ElbowAngle;
  double CurElevatorHeight;
  double CurElbowAngle;

};
