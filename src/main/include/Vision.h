#pragma once

#include <iostream>
#include <photon/PhotonUtils.h>
#include <photon/PhotonCamera.h>
#include <frc/XboxController.h>
#include <units/angle.h>
#include <units/length.h>
#include <frc/controller/PIDController.h>
#include <frc/drive/DifferentialDrive.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"



class Vision {
    private:
        photon::PhotonCamera camera{VisionConstants::cameraOne};
        frc::XboxController xboxController{0};

        frc::PIDController forwardController{VisionConstants::VISION_LINEAR_P, 0.0, VisionConstants::VISION_LINEAR_D};
        frc::PIDController turnController{VisionConstants::VISION_ANGULAR_P, 0.0, VisionConstants::VISION_ANGULAR_D};

        double forwardSpeed;
        double rotationSpeed;

        int aprilTagID;

        photon::PhotonPoseEstimator m_poseEstimator{
            frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
            photon::MULTI_TAG_PNP_ON_RIO, std::move(camera),
            frc::Transform3d{}};

    public:
        Vision() {
           frc::SmartDashboard::PutData("forwardPID", &forwardController); 
           frc::SmartDashboard::PutData("turnPID", &turnController); 

        }

        void SetPipelineIndex(int pipelineIndex) {
            if (pipelineIndex != camera.GetPipelineIndex()) {
                camera.SetPipelineIndex(pipelineIndex);
            }
        }

        double GetForwardSpeed() {
            return forwardSpeed;
        }

        double GetRotationSpeed() {
            return rotationSpeed;
        }

        int GetAprilTagID() {
            return aprilTagID;
        }

        bool NotePickupEnabled() {
            return xboxController.GetAButton();
        }

        bool StereoShotEnabled() {
            return xboxController.GetBButton();
        }

        void VisionPeriodic() {
            frc::SmartDashboard::PutBoolean("NotePickupEnabled", NotePickupEnabled());
            frc::SmartDashboard::PutBoolean("StereoShotEnabled", StereoShotEnabled());
            
            frc::SmartDashboard::PutNumber("AprilTagID", aprilTagID);
            frc::SmartDashboard::PutNumber("ForwardSpeed", forwardSpeed);
            frc::SmartDashboard::PutNumber("RotationSpeed", rotationSpeed);

            aprilTagID = -1;

             if (NotePickupEnabled()) {
                // Vision-alignment mode
                // Query the latest result from PhotonVision
                SetPipelineIndex(VisionConstants::colorPipeline);
                const auto& result = camera.GetLatestResult();

                if (result.HasTargets()) {
                    // First calculate range
                    units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
                        VisionConstants::CAMERA_HEIGHT, 
                        VisionConstants::TARGET_HEIGHT, 
                        VisionConstants::CAMERA_PITCH,
                        units::degree_t{result.GetBestTarget().GetPitch()});

                    // Use this range as the measurement we give to the PID controller.
                    // -1.0 required to ensure positive PID controller effort _increases_
                    // range
                    forwardSpeed = -forwardController.Calculate(range.value(),
                                                    VisionConstants::NOTE_GOAL_RANGE_METERS.value());

                    // Also calculate angular power
                    // -1.0 required to ensure positive PID controller effort _increases_ yaw
                    rotationSpeed =
                        -turnController.Calculate(result.GetBestTarget().GetYaw(), 0);
                } 
            } else if (StereoShotEnabled()) {
                SetPipelineIndex(VisionConstants::aprilTagPipeline);
                const auto& result = camera.GetLatestResult();

                if (result.HasTargets()) {
                    // First calculate range
                    units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
                        VisionConstants::CAMERA_HEIGHT, 
                        VisionConstants::TARGET_HEIGHT, 
                        VisionConstants::CAMERA_PITCH,
                        units::degree_t{result.GetBestTarget().GetPitch()});
                    
                    //Range should help us decide how to angle the shooter and how fast it should be running.
                    rotationSpeed =
                        -turnController.Calculate(result.GetBestTarget().GetYaw(), 0);
                    forwardSpeed = 0;
                    aprilTagID = result.GetBestTarget().GetFiducialId();
                }
            } else {
                // Manual Driver Mode
                forwardSpeed = 0;
                rotationSpeed = 0;
            }

            // Use our forward/turn speeds to control the drivetrain
            //drive.ArcadeDrive(forwardSpeed, rotationSpeed);
        }
        
        std::optional<photon::EstimatedRobotPose> Update(frc::Pose2d estimatedPose) {

            m_poseEstimator.SetReferencePose(frc::Pose3d(estimatedPose));
            
            return m_poseEstimator.Update();
        }

};