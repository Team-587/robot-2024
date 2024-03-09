// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <rev/CANSparkFlex.h>

#define HAVEINTAKE
//#define HAVEBEAMBREAK

class ShooterIntake : public frc2::SubsystemBase {
 public:
  ShooterIntake();

  void setIntakeStart();

  //void setIntakeStart(double leftXValue, double rightXValue);

  void setIntakeStop();

  void setBeginShooter(); 

  void setIntakeVelocity(double velocity);

  void setShooterVelocity(double velocity);

  bool getIntakeSensorState() { return !intakeSwitch.Get(); };
  #ifdef HAVEBEAMBREAK
  bool getFrontSensorState() { return !frontSwitch.Get(); };
  bool getBackSensorState() { return !backSwitch.Get(); };
  #endif
  void setIntakeShoot();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  
  static constexpr double ShortShootVelocity{0.55};
  static constexpr double LongShootVelocity{0.7};
  static constexpr double StopShootVelocity{0.0};

  static constexpr double intakeVelocity{0.2};
  static constexpr double slowIntakeVelocity{0.1};
  static constexpr double backwardsIntakeVelocity{-0.1};
  static constexpr double intakeShootVelocity{0.6};

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

 #ifdef HAVEBEAMBREAK
 frc::DigitalInput frontSwitch;
 frc::DigitalInput backSwitch;
 #endif

//Used to turn on intake
 bool startIntake;
 bool startIntakeShoot;

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
