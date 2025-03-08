// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <commands/UpdateLEDCommand.h>
#include <subsystems/LEDSubsystem.h>

frc::Timer m_ledTimer;
RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_Led.SetDefaultCommand(UpdateLEDCommand(&m_Led,&m_driverController,&m_ledTimer).ToPtr());

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  
}


