
// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of

// the WPILib BSD license file in the root directory of this project.



#include "commands/UpdateLEDCommand.h"

#include <subsystems/LEDSubsystem.h>

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Timer.h>
#include <string_view>
#include <ranges>

#include <array>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>


UpdateLEDCommand::UpdateLEDCommand(LEDSubsystem* LED, frc::Joystick* joystick, frc::Timer* m_ledTimer): m_Led(LED), m_DriveController(joystick), m_ledTimer(m_ledTimer) {

  // Use addRequirements() here to declare subsystem dependencies.
  //m_led.Start();
  AddRequirements(m_Led);
  //m_led.SetLength(121);
  

}



// Called when the command is initially scheduled.

void UpdateLEDCommand::Initialize() {
  m_ledTimer->Reset();
  m_ledTimer->Start();
  
}



// Called repeatedly when this Command is scheduled to run

void UpdateLEDCommand::Execute() {
  // for (int i = 1; i < 101; i++) {
  //   m_Led->SetLedColor(0,0,255,i,121);
  //   m_Led->SetLedColor(0,0,0,i-1,121);
    
  // }
//m_Led->IdleMove(0,0,255,1,21);



// frc::LEDPattern red = frc::LEDPattern::Solid(frc::Color::kBlue);
// red.ApplyTo(m_ledBuffer);
// m_led.SetData(m_ledBuffer);






//  // Our LED strip has a density of 120 LEDs per meter
  units::meter_t kLedSpacing{1 / 120.0};


  std::array<frc::AddressableLED::LEDData, 120> m_buffer;


  //std::array<frc::AddressableLED::LEDData, 120> m_buffer;

// Create a view for the left section (first 60 LEDs)
auto m_left = std::ranges::take_view(m_buffer, 60);

// Create a reversed view for the right section (last 60 LEDs)
auto m_right = std::ranges::reverse_view(std::ranges::drop_view(m_buffer, 60));


  
  //std::vector<frc::AddressableLED::LEDData> right_buffer(m_right.begin(), m_right.end());
  //std::vector<frc::AddressableLED::LEDData> left_buffer(m_left.begin(), m_left.end());
  //std::vector<frc::AddressableLED::LEDData> right_buffer(m_right.begin(), m_right.end());

   
     
  if (m_DriveController->GetRawButton(1)) {
      frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(255, 128);
      frc::LEDPattern m_scrollingRainbow =
      m_rainbow.ScrollAtAbsoluteSpeed(1_mps, kLedSpacing);
      m_scrollingRainbow.ApplyTo(m_ledBuffer);
      m_led.SetData(m_ledBuffer);




  } //else if (m_DriveController->GetRawButton(2)) {
  //   std::array<frc::Color, 2> colors{frc::Color::kRed, frc::Color::kBlue};
  //   frc::LEDPattern gradient = frc::LEDPattern::Gradient(frc::LEDPattern::GradientType::kContinuous, colors);
  //   gradient.ApplyTo(m_ledBuffer);  
  //   m_led.SetData(m_ledBuffer);
  //} 



  else if (m_DriveController->GetRawButton(2)) {
    frc::LEDPattern blue = frc::LEDPattern::Solid(frc::Color::kBlue);
  blue.ApplyTo(m_buffer);
  m_led.SetData(m_buffer);
  

  }
  else if (m_DriveController->GetRawButton(3)) {
    std::array<std::pair<double, frc::Color>, 2> maskSteps{std::pair{0.0, frc::Color::kWhite},
                                                  std::pair{0.1, frc::Color::kBlack}};
  frc::LEDPattern base = frc::LEDPattern::Rainbow(255, 255);
  frc::LEDPattern mask = frc::LEDPattern::Steps(maskSteps).ScrollAtRelativeSpeed(units::hertz_t{0.25});

  frc::LEDPattern pattern =  base.Mask(mask);
  pattern.ApplyTo(m_ledBuffer);  
  m_led.SetData(m_ledBuffer);




  } //else if (m_DriveController->GetRawButton(4)) {
  //   std::array<frc::Color, 2> colors{frc::Color::kGreen, frc::Color::kBlue};
  //   frc::LEDPattern base = frc::LEDPattern::Gradient(frc::LEDPattern::GradientType::kDiscontinuous, colors);
  //  frc:: LEDPattern pattern = base.Breathe(2_s);

  //   // Apply the LED pattern to the data buffer
  //   pattern.ApplyTo(m_ledBuffer);

  //   // Write the data to the LED strip
  //   m_led.SetData(m_ledBuffer);
  // }
  else if (m_DriveController->GetRawButton(4)) {
    frc::LEDPattern green = frc::LEDPattern::Solid(frc::Color::kRed);
    green.ApplyTo(m_buffer);
    m_led.SetData(m_buffer);
  }


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
