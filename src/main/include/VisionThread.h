#pragma once

#include <thread>
#include <unistd.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/mutex.h>

#include "subsystems/VisionSubsystem.h"

class VisionThread {
  public:

    //VisionThread(VisionThread&&) { };
    VisionThread(std::string Name, VisionSubsystem *pVisionSubsystem) {
      m_Name = Name;
      m_pVisionSubsystem = pVisionSubsystem;
    }

    void Start() {
      std::thread m_thread(&VisionThread::VisionProcess, this);
      m_thread.detach();
    }

    void Enable(bool Enable) {
      m_Enable = Enable;
    }

    photon::PhotonTrackedTarget* GetTarget() { 
      //std::scoped_lock lock{mutex};
      return m_pTarget; 
    }

  private:

    void VisionProcess() {
      while(true) {
        if(m_Enable) {
          std::optional<photon::PhotonTrackedTarget> target = m_pVisionSubsystem->GetBestTarget();
          //frc::SmartDashboard::PutBoolean(m_Name + " Targets", target.has_value());
          std::cout << m_Name << "Targeting\n";
          if(target.has_value()) {
            SetTarget(&target.value());
          } else {
            SetTarget(nullptr);
          }
        }
        //usleep(1000000); 
        sleep(300);
      }
    }

    void SetTarget(photon::PhotonTrackedTarget *pTarget) {
      //std::scoped_lock lock{mutex};
      m_pTarget = pTarget;
    }

    //wpi::mutex mutex;
    bool m_Enable = false;
    std::string m_Name;
    VisionSubsystem *m_pVisionSubsystem;
    photon::PhotonTrackedTarget *m_pTarget;
};