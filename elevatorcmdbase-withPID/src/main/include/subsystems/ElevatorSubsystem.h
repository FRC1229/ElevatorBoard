// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <rev/SparkMax.h>
#include <algorithm>
#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/trajectory/TrapezoidProfile.h>

double kP=0.02;
double kI=0.0;
double kD=0.00;
units::volt_t kS = 4_V;
units::volt_t kG = 2_V;
auto kV = 2_V/(0.5_mps);
auto kA = 2_V/(0.5_mps_sq);


class ElevatorSubsystem : public frc2::SubsystemBase {
 public:
  ElevatorSubsystem();
    rev::spark::SparkMax m_ElevatorMotorBottom;
    rev::spark::SparkMax m_ElevatorMotorTop;
    rev::spark::SparkAbsoluteEncoder m_ElevatorEncoderBottom;
    rev::spark::SparkAbsoluteEncoder m_ElevatorEncoderTop;
    


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  double readEncoder(double encodervalue);
  void SetElevatorSpeed(double speed);
  void SetInverted();
  void HomePosition();
  void L1CoralPosition();
  void L2CoralPosition();
  void L3CoralPosition();
  void L4CoralPosition();

  //CHANGE THESE PLEASE

  

    // Creates a PIDController with gains kP, kI, and kD
  frc::ProfiledPIDController<units::meters> m_controller(
    0.02, kI, kD, 
    frc::TrapezoidProfile<units::meters>::Constraints{0.3_mps, 0.3_mps_sq});
  frc::ElevatorFeedforward m_feedforward(kS, kG, kV, kA);
  frc::TrapezoidProfile<units::meters>::State goal;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  double kP=0.02;
};