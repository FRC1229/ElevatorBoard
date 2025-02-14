// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/HomePosition.cpp"
#include "commands/L1CoralPosition.cpp"
#include "commands/L2CoralPosition.cpp"
#include "commands/L3CoralPosition.cpp"
#include "commands/L4CoralPosition.cpp"
#include <frc2/command/button/JoystickButton.h>


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  
  frc2::JoystickButton(&r_controller, frc::XboxController::Button::kA).OnTrue(HomePosition(&m_elevatorSubsystem).ToPtr());
  frc2::JoystickButton(&r_controller, frc::XboxController::Button::kB).OnTrue(L1CoralPosition(&m_elevatorSubsystem).ToPtr());
  frc2::JoystickButton(&r_controller, frc::XboxController::Button::kY).OnTrue(L2CoralPosition(&m_elevatorSubsystem).ToPtr());
  frc2::JoystickButton(&r_controller, frc::XboxController::Button::kX).OnTrue(L3CoralPosition(&m_elevatorSubsystem).ToPtr());
}
