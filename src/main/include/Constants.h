// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angular_acceleration.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */


//#define SWERVEBASE


namespace DriveConstants {

constexpr int kLEDsideLength = 26;
constexpr int kLEDbackLength = 36; 
constexpr int kLEDtotalLength = kLEDsideLength + kLEDsideLength + kLEDbackLength;
constexpr int kLEDPort = 0;

constexpr int kLeftTentacleMotorPort = 2;
constexpr int kRightTentacleMotorPort = 3;

constexpr int kIntakeMotorPort = 4;
constexpr int kOutakeMotorPort = 5;
constexpr int kIntakeSwitchPort = 0; //DIO port
constexpr int kfrontSwitchPort = 2;
constexpr int kbackSwitchPort = 3;

constexpr int kElevatorMotorPort = 6;
constexpr int kElbowAMotorPort = 7;
constexpr int kElbowBMotorPort = 8;

constexpr int kFrontLeftDriveMotorPort = 18;
constexpr int kRearLeftDriveMotorPort = 16;
constexpr int kFrontRightDriveMotorPort = 11;
constexpr int kRearRightDriveMotorPort = 13;

constexpr int kFrontLeftTurningMotorPort = 17;
constexpr int kRearLeftTurningMotorPort = 15;
constexpr int kFrontRightTurningMotorPort = 12;
constexpr int kRearRightTurningMotorPort = 14;

constexpr int kFrontLeftAbsoluteEncoderPort = 23;
constexpr int kFrontRightAbsoluteEncoderPort = 22;
constexpr int kRearLeftAbsoluteEncoderPort = 24;
constexpr int kRearRightAbsoluteEncoderPort = 21;

constexpr bool kFrontLeftTurningEncoderReversed = false;
constexpr bool kRearLeftTurningEncoderReversed = false;
constexpr bool kFrontRightTurningEncoderReversed = false;
constexpr bool kRearRightTurningEncoderReversed = false;

constexpr bool kFrontLeftDriveEncoderReversed = false;
constexpr bool kRearLeftDriveEncoderReversed = false;
constexpr bool kFrontRightDriveEncoderReversed = false;
constexpr bool kRearRightDriveEncoderReversed = false;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The SysId tool provides a convenient
// method for obtaining these values for your robot.
constexpr auto ks = 1_V;
constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
constexpr double kPFrontLeftVel = 0.5;
constexpr double kPRearLeftVel = 0.5;
constexpr double kPFrontRightVel = 0.5;
constexpr double kPRearRightVel = 0.5;
}  // namespace DriveConstants

namespace ModuleConstants {
constexpr int kEncoderCPR = 1024;
constexpr double kWheelDiameterMeters = 0.15;
constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * std::numbers::pi) /
    static_cast<double>(kEncoderCPR);

constexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (std::numbers::pi * 2) / static_cast<double>(kEncoderCPR);

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 4.5_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kCoDriverControllerPort = 1;
}  // namespace OIConstants

namespace ShooterIntakeConstants {
    static constexpr double intakeVelocity{0.4};
    static constexpr double intakeShootVelocity{0.6};
}

namespace VisionConstants {
    const std::string cameraNote = "Microsoft_LifeCam_HD-3000";
    const std::string cameraAprilTag = "Microsoft_LifeCam_HD-3001";
    const int colorPipeline = 0;

    const int aprilTagPipeline = 1;

    const int blueAprilTag = 7;
    const int redAprilTag = 4;

    const units::meter_t CAMERA_HEIGHT_NOTE = 24_in;
    const units::meter_t TARGET_HEIGHT_NOTE = 5_ft;
    const units::radian_t CAMERA_PITCH_NOTE = 0_deg;

    const units::meter_t CAMERA_HEIGHT_APRILTAG = 7.2_in;
    const units::meter_t TARGET_HEIGHT_APRILTAG = 53.88_in;
    const units::radian_t CAMERA_PITCH_APRILTAG = 23_deg;

    const units::meter_t CAMERA_HEIGHT_AT = 24_in;
    const units::meter_t TARGET_HEIGHT_AT = 5_ft;
    const units::radian_t CAMERA_PITCH_AT = 0_deg;

    const units::meter_t NOTE_GOAL_RANGE_METERS = .5_ft;
    const units::meter_t STEREO_GOAL_RANGE_METERS = 3_ft;

    const double VISION_LINEAR_P = 0.1;
    const double VISION_LINEAR_D = 0.0;

    const double VISION_ANGULAR_P = 0.1;
    const double VISION_ANGULAR_D = 0.0;

    const uint64_t MAX_TARGET_LATENCY = 1 * 1000;
}
