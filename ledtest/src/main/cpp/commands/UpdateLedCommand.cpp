
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of

// the WPILib BSD license file in the root directory of this project.



#include "commands/UpdateLEDCommand.h"

#include <subsystems/LEDSubsystem.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Timer.h>



UpdateLEDCommand::UpdateLEDCommand(LEDSubsystem* LED, frc::Joystick* joystick, frc::Timer* m_ledTimer): m_Led(LED), m_DriveController(joystick), m_ledTimer(m_ledTimer) {

  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_Led);

}



// Called when the command is initially scheduled.

void UpdateLEDCommand::Initialize() {}



// Called repeatedly when this Command is scheduled to run

void UpdateLEDCommand::Execute() {


  

   // if (m_DriveController->GetRawButton(2)) {

      
   //    m_ledTimer->Start();
   //    if (m_ledTimer->Get().value() < 0.5) {
   //       m_Led->SetLedColor(255,0,0,121);
   //    } 
   //    if (m_ledTimer->Get().value() > 0.5 && m_ledTimer->Get().value() < 1) {
   //       m_Led->SetLedColor(0,255,0,121);
   //       m_ledTimer->Reset();
   //    } 


   // } else {
   //    m_ledTimer->Stop();
      
   // }


  //   else if(m_Elevator->readEncoder()>0.903 && m_Elevator->readEncoder()<0.907){

  //     m_Led->SetLedColor(0,255,0,121);

  //   }



  //   else if(m_Elevator->readEncoder()>0.003 && m_Elevator->readEncoder()<0.007){

  //    m_Led->SetLedColor(0,0,255,121);


  //   }
  //   else if(m_Elevator->readEncoder()>0.768 && m_Elevator->readEncoder()<0.772){
  //     m_Led->SetLedColor(255,255,0,121);
  //    }
  //   else if(m_Elevator->readEncoder()>0.412 && m_Elevator->readEncoder()<0.416){
  //     m_Led->SetLedColor(0,255,255,121);
  //    }
    // }
    // else if(m_algae->GetAngle()>18 && m_algae->GetAngle()<22){
    //  m_Led->SetLedColor(255,255,0,121);
    // }
    // else if(m_algae->GetAngle()>0.5 && m_algae->GetAngle()<4){
    //        m_Led->SetLedColor(102,0,102,121);

    // }

    // else if(m_DriveController->GetRawAxis(2)>0.10){
    // m_Led->SetLedColor(0,255,255,60);
    // }

    // else if(m_DriveController->GetRawAxis(2)>0.10){
    // m_Led->SetLedColor(255,255,255,120);
    // }
    
 }

// Called once the command ends or is interrupted.
void UpdateLEDCommand::End(bool interrupted) {

}

// Returns true when the command should end.
bool UpdateLEDCommand::IsFinished() {
  return false;
}
