// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkFlex.h>

#define HAVEINTAKE

class ShooterIntake : public frc2::SubsystemBase {
 public:
  ShooterIntake();

  void setIntakeStart();

  void setIntakeStop();

  void setBeginShooter(); 

  void setIntakeVelocity(double velocity);

  void setShooterVelocity(double velocity);

  bool getIntakeSensorState() { return !intakeSwitch.Get(); };
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  static constexpr double ShortShootVelocity{0.55};
  static constexpr double LongShootVelocity{0.7};
  static constexpr double StopShootVelocity{0.0};

  static constexpr double intakeVelocity{0.21};
  static constexpr double intakeShootVelocity{0.45};

  bool getEndgame() {
    return endgameToggle;
  }

  void setEndgame() {
    endgameToggle = !endgameToggle;
  }

 private:

 //Setting up states for intake/shooting cycle
 enum  stateType{
    STOP,
    INTAKE,
    HAVENOTE,
    SHOOTSETUP,
    SHOOTING,
    //REVERSE
 };
 //end game
 bool endgameToggle = false;
 
 //Creating variable for state of motors
 stateType stateVar;

#ifdef HAVEINTAKE
//Seting up motors
 rev::CANSparkFlex intakeMotor;
 rev::CANSparkFlex outakeMotor;

//Using PID to set motor speed for intake/outake
 rev::SparkPIDController motorSpeedPID;
#endif

//Declaring the switch to detect notes
 frc::DigitalInput intakeSwitch;

//Used to turn on intake
 bool startIntake;

//Used to turn off intake
 bool stopIntake;

// Starts Shooter
 bool beginShooter;

//Store velocity
double shooterVelocity;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

//How many times the loops get runs before shooting.
int delayCount;
};
