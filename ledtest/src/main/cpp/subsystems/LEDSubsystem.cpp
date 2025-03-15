// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDSubsystem.h"





LEDSubsystem::LEDSubsystem()
{
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
    
};

// This method will be called once per scheduler run
void LEDSubsystem::Periodic() {


}

void LEDSubsystem::SetLedColor(int r, int g, int b, int start, int end){
   for(int i = start; i<end;i++){
        m_ledBuffer[i].SetRGB(r,g,b);
    }
    m_led.SetData(m_ledBuffer);
}

void LEDSubsystem::IdleMove(int r, int g, int b, int start, int end){
    for (int i = start; i <= end; i++){
        m_ledBuffer[i].SetRGB(r,g,b);

        if (end < 121){
            start+=1;
            end+=1;
        }
        else {
            start-=1;
            end-=1;
        }
        m_ledBuffer[i-1].SetRGB(0,0,0);
    }
    m_led.SetData(m_ledBuffer);
}

    


