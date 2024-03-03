// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Lights.h"
#include <frc/DriverStation.h>
#include <frc/DigitalInput.h>
#include "subsystems/ShooterIntake.h"

Lights::Lights()
{
  for (int i = 0; i < DriveConstants::kLEDtotalLength; i = i + 3) {
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
  m_led.SetLength(DriveConstants::kLEDtotalLength);
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
     if(temp == true) {
        for (int i = 0; i < DriveConstants::kLEDsideLength; i++)
        {
            haveNoteLight = temp;
            LEDsideArray[i].SetRGB(167, 3, 255);
        }

        for (int i = 0; i < DriveConstants::kLEDbackLength; i++)
        {
            haveNoteLight = temp;
            LEDbackArray[i].SetRGB(167, 3, 255);
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
            for (int i = 0; i < DriveConstants::kLEDsideLength; i = i + 2)
            {
                LEDsideArray[i].SetRGB(0, 133, 202);
                LEDsideArray[i + 1].SetRGB(255, 234, 0);
                blueActive = false;
            }

            for (int i = 0; i < DriveConstants::kLEDbackLength; i = i + 2)
            {
                LEDbackArray[i].SetRGB(0, 133, 202);
                LEDbackArray[i + 1].SetRGB(255, 234, 0);
                blueActive = false;
            }               
        } else {
            for (int i = 0; i < DriveConstants::kLEDsideLength; i = i + 2)
            {
                LEDsideArray[i].SetRGB(255, 234, 0);
                LEDsideArray[i + 1].SetRGB(0, 133, 202);
                blueActive = true;
            }

            for (int i = 0; i < DriveConstants::kLEDbackLength; i = i + 2)
            {
                LEDbackArray[i].SetRGB(255, 234, 0);
                LEDbackArray[i + 1].SetRGB(0, 133, 202);
                blueActive = true;
            }      
        }
    }
    else {
        double matchTime = (double)frc::DriverStation::GetMatchTime();
        double totalMatchTime = 135;
        bool autoEnabled = false;
        bool endgame = false;
        if(frc::DriverStation::IsAutonomousEnabled()){
            totalMatchTime = 15;
            autoEnabled = true;
        }else{
            autoEnabled = false;
            if(matchTime <= 15){
                totalMatchTime = 15;
                endgame = true;
            }
            else{
                endgame = false;
            }
        }
        int numLED = ((matchTime / totalMatchTime) * (double)DriveConstants::kLEDsideLength);
        int numLEDback = ((matchTime / totalMatchTime) * (double)DriveConstants::kLEDbackLength); 

        for (int i = 0; i < DriveConstants::kLEDsideLength; i++)
        {
            if(i < numLED) {
                if(endgame == true){
                    LEDsideArray[i].SetRGB(255, 0, 0);
                }else {
                LEDsideArray[i].SetRGB(0, 133, 202);
                }
            } else {
                    LEDsideArray[i].SetRGB(255, 234, 0);
            }
        }
        
        int indexFront = DriveConstants::kLEDbackLength / 2 - 1;
        int indexBack = DriveConstants::kLEDbackLength / 2;
        for (int i = 0; i < DriveConstants::kLEDbackLength; i = i + 2)
        {
           if(i < numLEDback) {
            if(endgame == true){
                LEDbackArray[indexFront].SetRGB(255, 0, 0);
                LEDbackArray[indexBack].SetRGB(255, 0, 0);
             }else {
                LEDbackArray[indexFront].SetRGB(0, 133, 202);
                LEDbackArray[indexBack].SetRGB(0, 133, 202);
            }
            }else {
                LEDbackArray[indexFront].SetRGB(255, 234, 0);
                LEDbackArray[indexBack].SetRGB(255, 234, 0);
            }
            indexFront--;
            indexBack++;
        }
        
    }
  

  int index = DriveConstants::kLEDtotalLength - 1;
  int indexmiddle = DriveConstants::kLEDsideLength;

  for (int i = 0; i < DriveConstants::kLEDsideLength; i++) {
    topLEDArray[i] = LEDsideArray[i];
    topLEDArray[index] = LEDsideArray[i];
    index--;
  }

  for (int i = 0; i < DriveConstants::kLEDbackLength; i++) {
    topLEDArray[indexmiddle] = LEDbackArray[i];
    indexmiddle++;
  }

  m_led.SetData(topLEDArray);
  m_led.Start();

}
