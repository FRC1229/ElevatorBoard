// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem() = default;

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include <Constants.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <units/time.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>


ElevatorSubsystem::ElevatorSubsystem():
m_ElevatorMotorBottom(11, rev::spark::SparkMax::MotorType::kBrushless),
m_ElevatorMotorTop(12, rev::spark::SparkMax::MotorType::kBrushless),
m_ElevatorEncoderBottom(m_ElevatorMotorBottom.GetAbsoluteEncoder()),
m_ElevatorEncoderTop(m_ElevatorMotorTop.GetAbsoluteEncoder())
{

}



void ElevatorSubsystem::SetInverted() {
    m_ElevatorMotorBottom.SetInverted(true);
    m_ElevatorMotorTop.SetInverted(false);
}



// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {

}
double ElevatorSubsystem::readEncoder(double encodervalue){
    return m_ElevatorEncoderTop.GetPosition();
    return m_ElevatorEncoderBottom.GetPosition();
    // frc::SmartDashboard::PutNumber("shoulder", shoulder.GetDistance());
}

void ElevatorSubsystem::SetElevatorSpeed(double speed){
    // m_ElevatorMotor.Set(speed);
} 

void ElevatorSubsystem::HomePosition(){
    // Set the SetPoints here
    m_ElevatorMotorBottom.SetInverted(true);
    m_ElevatorMotorTop.SetInverted(false);

    double setPoint = 0.5;
    double encoderValueTop = m_ElevatorEncoderBottom.GetPosition();
    double encoderValueBottom = m_ElevatorEncoderTop.GetPosition();
    double elevatorPosition = (encoderValueTop/0.786)+2;
    double autoAccelFactor = 0;
    double autospeed = 0;

    if (autoAccelFactor < 1)
    {

        
    }

}

void ElevatorSubsystem::L1CoralPosition(){
    m_ElevatorMotorBottom.SetInverted(true);
    m_ElevatorMotorTop.SetInverted(false);
    // Set the SetPoints here
    double TopElevatorSetPoint = 1;
    double BottomElevatorSetPoint = 1;

    double degrees = readEncoder(m_ElevatorEncoderTop.GetPosition());
    double volt = L1CoralTopElevatorPID.Calculate(degrees, TopElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorTop.SetVoltage(units::volt_t{volt});

     double degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    double volt = L1CoralBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});
}

void ElevatorSubsystem::L2CoralPosition(){
    m_ElevatorMotorBottom.SetInverted(true);
    m_ElevatorMotorTop.SetInverted(false);
    // Set the SetPoints here
    double TopElevatorSetPoint = 1;
    double BottomElevatorSetPoint = 1;

    double degrees = readEncoder(m_ElevatorEncoderTop.GetPosition());
    double volt = L2CoralTopElevatorPID.Calculate(degrees, TopElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorTop.SetVoltage(units::volt_t{volt});

     double degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    double volt = L2CoralBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});
}
void ElevatorSubsystem::L3CoralPosition(){
    m_ElevatorMotorBottom.SetInverted(true);
    m_ElevatorMotorTop.SetInverted(false);
    // Set the SetPoints here
    double TopElevatorSetPoint = 1;
    double BottomElevatorSetPoint = 1;

    double degrees = readEncoder(m_ElevatorEncoderTop.GetPosition());
    double volt = L3CoralTopElevatorPID.Calculate(degrees, TopElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorTop.SetVoltage(units::volt_t{volt});

     double degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    double volt = L3CoralBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});
}

void ElevatorSubsystem::L4CoralPosition(){
    m_ElevatorMotorBottom.SetInverted(true);
    m_ElevatorMotorTop.SetInverted(false);
    // Set the SetPoints here
    double TopElevatorSetPoint = 1;
    double BottomElevatorSetPoint = 1;

    double degrees = readEncoder(m_ElevatorEncoderTop.GetPosition());
    double volt = L4CoralTopElevatorPID.Calculate(degrees, TopElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorTop.SetVoltage(units::volt_t{volt});

     double degrees = readEncoder(m_ElevatorEncoderBottom.GetPosition());
    double volt = L4CoralBottomElevatorPID.Calculate(degrees, BottomElevatorSetPoint);
    frc::SmartDashboard::PutNumber("volt calc", volt);
    m_ElevatorMotorBottom.SetVoltage(units::volt_t{volt});
}