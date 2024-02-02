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
#include <frc/shuffleboard/ShuffleBoard.h>

#include "Constants.h"



class Vision {
    private:
        photon::PhotonCamera camera{VisionConstants::cameraOne};
        //frc::XboxController xboxController{0};
        frc::XboxController xboxController{OIConstants::kDriverControllerPort};

        frc::PIDController forwardController{VisionConstants::VISION_LINEAR_P, 0.0, VisionConstants::VISION_LINEAR_D};
        frc::PIDController turnController{VisionConstants::VISION_ANGULAR_P, 0.0, VisionConstants::VISION_ANGULAR_D};

        double forwardSpeed;
        double rotationSpeed;

        int aprilTagID;

        /*photon::PhotonPoseEstimator m_poseEstimator{
            frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
            photon::MULTI_TAG_PNP_ON_RIO, std::move(camera),
            frc::Transform3d{}};
*/
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
                std::cout << "I am here\n";
                // Vision-alignment mode
                // Query the latest result from PhotonVision
                //SetPipelineIndex(VisionConstants::colorPipeline);
                SetPipelineIndex(0);
                std::cout << "I am here 2" << camera.GetCameraName() << "\n";
                std::cout << "I am here 2\n";
                const auto& result = camera.GetLatestResult();
                std::cout << "I am here 3\n";

                frc::SmartDashboard::PutBoolean("HasTargets", result.HasTargets());

                if (result.HasTargets()) {
                    std::cout << "I am here 4\n";
                    // First calculate range
                    units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
                        VisionConstants::CAMERA_HEIGHT, 
                        VisionConstants::TARGET_HEIGHT, 
                        VisionConstants::CAMERA_PITCH,
                        units::degree_t{result.GetBestTarget().GetPitch()});
                        std::cout << "I am here 5\n";

                    // Use this range as the measurement we give to the PID controller.
                    // -1.0 required to ensure positive PID controller effort _increases_
                    // range
                    /*forwardSpeed = -forwardController.Calculate(range.value(),
                                                    VisionConstants::NOTE_GOAL_RANGE_METERS.value());*/
                    forwardSpeed = 0;
                    std::cout << "I am here 6\n";
                    // Also calculate angular power
                    // -1.0 required to ensure positive PID controller effort _increases_ yaw
                    rotationSpeed = 
                        -turnController.Calculate(result.GetBestTarget().GetYaw(), 0);

                    std::cout << "I am here 7\n";
                } else {
                    forwardSpeed = 0;
                    rotationSpeed = 0;
                }
            } else if (StereoShotEnabled()) {
                std::cout << "I am here 8\n";

                SetPipelineIndex(VisionConstants::aprilTagPipeline);
                const auto& result = camera.GetLatestResult();

                std::cout << "I am here 9\n";

                if (result.HasTargets()) {
                    std::cout << "I am here 10\n";
                    // First calculate range
                    units::meter_t range = photon::PhotonUtils::CalculateDistanceToTarget(
                        VisionConstants::CAMERA_HEIGHT, 
                        VisionConstants::TARGET_HEIGHT, 
                        VisionConstants::CAMERA_PITCH,
                        units::degree_t{result.GetBestTarget().GetPitch()});
                    std::cout << "I am here 11\n";
                    
                    //Range should help us decide how to angle the shooter and how fast it should be running.
                    rotationSpeed =
                        -turnController.Calculate(result.GetBestTarget().GetYaw(), 0);
                    forwardSpeed = 0;
                    aprilTagID = result.GetBestTarget().GetFiducialId();

                    std::cout << "I am here 12\n";
                }
            } else {
                // Manual Driver Mode
                forwardSpeed = 0;
                rotationSpeed = 0;
                SetPipelineIndex(VisionConstants::aprilTagPipeline);
            }

            // Use our forward/turn speeds to control the drivetrain
            //drive.ArcadeDrive(forwardSpeed, rotationSpeed);
        }
        
        /*std::optional<photon::EstimatedRobotPose> Update(frc::Pose2d estimatedPose) {

            //m_poseEstimator.SetReferencePose(frc::Pose3d(estimatedPose));
            
            return NULL;//m_poseEstimator.Update();
        }*/

};