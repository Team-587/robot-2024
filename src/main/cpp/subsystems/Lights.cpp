// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lights.h"
#include <frc/DriverStation.h>
#include <frc/DigitalInput.h>

Lights::Lights()
{
  for (int i = 0; i < DriveConstants::kLEDLength * 2; i = i + 3) {
    //topLEDArray[i].SetRGB(0, 255, 0);
    //0, 133, 202
    //255, 234, 0

    topLEDArray[i].SetRGB(2, 5, 97);
    topLEDArray[i + 1].SetRGB(0, 133, 202);
    topLEDArray[i + 2].SetRGB(252, 186, 3);
    //ledArray[i + 1].SetRGB(0, 133, 202);
    //ledArray[i + 2].SetRGB(3, 8, 143);
  }
  
  ledLoopCount = 9;
  disableLoopCount = 2;
  haveNoteLight = false;
  m_led.SetLength(DriveConstants::kLEDLength * 2);
  m_led.SetData(topLEDArray);
  m_led.Start();
}

// This method will be called once per scheduler run
void Lights::Periodic() {

    ledLoopCount--;

    if(ledLoopCount > 0) {
        return;
    }

    ledLoopCount = 9;

    frc::DigitalInput intakeSwitch {DriveConstants::kIntakeSwitchPort + 1};
    bool temp = !intakeSwitch.Get();
     if(temp == true && haveNoteLight == false) {
        for (int i = 0; i < DriveConstants::kLEDLength; i++)
        {
            haveNoteLight = temp;
            ledArray[i].SetRGB(255, 0, 0);
        }
      
     } else if(temp == false && haveNoteLight == true){
        haveNoteLight = false;
     }

    else if(frc::DriverStation::IsDisabled() == true) {
        disableLoopCount--;

        if(disableLoopCount > 0){
            return;
        }

        disableLoopCount = 3;
        if(blueActive) {
            for (int i = 0; i < DriveConstants::kLEDLength; i = i + 2)
            {
                ledArray[i].SetRGB(0, 133, 202);
                ledArray[i + 1].SetRGB(255, 234, 0);
                blueActive = false;
            }            
        } else {
            for (int i = 0; i < DriveConstants::kLEDLength; i = i + 2)
            {
                ledArray[i].SetRGB(255, 234, 0);
                ledArray[i + 1].SetRGB(0, 133, 202);
                blueActive = true;
            }   
        }
    }
    else {
        double matchTime = (double)frc::DriverStation::GetMatchTime();
        double totalMatchTime = 135;
        if(frc::DriverStation::IsAutonomousEnabled()){
            totalMatchTime = 15;
        }
        int numLED = ((matchTime / totalMatchTime) * (double)DriveConstants::kLEDLength); 

        for (int i = 0; i < DriveConstants::kLEDLength; i++)
        {
            if(i < numLED) {
                ledArray[i].SetRGB(0, 133, 202);
            } else {
                ledArray[i].SetRGB(255, 234, 0);
            }
        }
        
    }
  

  int index = DriveConstants::kLEDLength * 2 - 1;

  for (int i = 0; i < DriveConstants::kLEDLength; i++) {
    topLEDArray[i] = ledArray[i];
    topLEDArray[index] = ledArray[i];
    index--;
  }

  m_led.SetData(topLEDArray);
  m_led.Start();

}
