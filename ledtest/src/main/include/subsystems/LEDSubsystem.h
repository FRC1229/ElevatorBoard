// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc/util/Color.h>
#include <frc/Timer.h>

static constexpr int kLength = 120;

extern frc::AddressableLED m_led;
extern std::array<frc::AddressableLED::LEDData, kLength>
    m_ledBuffer;

class LEDSubsystem : public frc2::SubsystemBase {
 public:
  LEDSubsystem();
  frc::AddressableLED m_led{8};
  std::array<frc::AddressableLED::LEDData, 121> m_ledBuffer; 
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;    
  frc2::CommandPtr RunPattern(frc::LEDPattern pattern);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
  void SetLedColor(int r, int g, int b, int start, int end);
  void IdleMove(int r, int g, int b, int start, int length);
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
