// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Commands/SetInverted.h"

SetInverted::SetInverted(ElevatorSubsystem* subsystem) : m_elevatorSubsystem(subsystem) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({subsystem});
}

// Called when the command is initially scheduled.
void SetInverted::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetInverted::Execute() {
  m_elevatorSubsystem->HomePosition();
}

// Called once the command ends or is interrupted.
void SetInverted::End(bool interrupted) {}

// Returns true when the command should end.
bool SetInverted::IsFinished() {
  return false;
}