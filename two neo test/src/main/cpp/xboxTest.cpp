// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/XboxController.h>
#include <rev/SparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <units/acceleration.h>
#include <frc/Servo.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <units/voltage.h>




int firstPixelHue = 0;
int kLength = 60;
int scrollOffset = 0;

using namespace rev::spark;
frc::Timer delayTimer;
frc::XboxController r_controller{0};
frc::AddressableLED m_led{0};
frc::Servo t_servo{1};
std::array<frc::AddressableLED::LEDData, 60>m_ledBuffer;


frc::PIDController Position1PID{1,0,0};
frc::PIDController Position2PID{1,0,0};

SparkMax m_max12{11, SparkMax::MotorType::kBrushless};


rev::spark::SparkAbsoluteEncoder max12(m_max12.GetAbsoluteEncoder());


Robot::Robot() {
  
}
void Robot::RobotPeriodic() {

}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // m_led.SetLength(kLength);
  // m_led.SetData(m_ledBuffer);
  // m_led.Start();
}


double readEncoder() {
  //return max11.GetPosition();
  return max12.GetPosition();
}

void Position1() {
  double setPoint = 1.0;

  double degrees = readEncoder();
  
  double volt = Position1PID.Calculate(degrees, setPoint);
  frc::SmartDashboard::PutNumber("ENCODER", degrees);
  frc::SmartDashboard::PutNumber("volt calc", volt);
  m_max12.SetVoltage(units::volt_t{volt});
  //m_max12.Set(volt);
  

}

void Position2() {
  double setPoint = 0.8;

  double degrees = readEncoder();
  double volt = Position2PID.Calculate(degrees, setPoint);
  frc::SmartDashboard::PutNumber("ENCODER", degrees);
  frc::SmartDashboard::PutNumber("volt calc", volt);
  m_max12.SetVoltage(units::volt_t{volt});
  //m_max12.Set(volt);

}

void Robot::TeleopPeriodic() {
 
  // for (int i = 0; i<kLength; i++) {
  //   int index = (i+scrollOffset)%kLength;
  //   if (index % 10 < 5) {
  //     m_ledBuffer[i].SetRGB(150,0,128);
  //   }else {
  //     m_ledBuffer[i].SetRGB(0,0,0);
  //   }
  // }

  // scrollOffset = (scrollOffset + 1) % kLength;

  // m_led.SetData(m_ledBuffer);
  
  
  
  
  // Only toggle rumble if A button is pressed
  
  if (r_controller.GetAButton()) {
    delayTimer.Start();
    if (delayTimer.Get().value() < 0.5) {
    r_controller.SetRumble(frc::XboxController::RumbleType::kLeftRumble, 1.0);
    //r_controller.SetRumble(frc::XboxController::RumbleType::kRightRumble, 1.0);
    }

    if (delayTimer.Get().value() >= 0.5 && delayTimer.Get().value() < 1.0) {
      r_controller.SetRumble(frc::XboxController::RumbleType::kLeftRumble, 0.0);
      //r_controller.SetRumble(frc::XboxController::RumbleType::kRightRumble, 0.2);
    }


    if (delayTimer.Get().value() >= 1.0) {
      delayTimer.Stop();
      delayTimer.Reset();
    }

  } else {
    delayTimer.Stop();
    delayTimer.Reset();
    r_controller.SetRumble(frc::XboxController::RumbleType::kLeftRumble, 0.0);
    //r_controller.SetRumble(frc::XboxController::RumbleType::kRightRumble, 0.2);

  }

  // if (r_controller.GetXButtonPressed()) {
  //   t_servo.SetAngle(0);
  // }
  // if (r_controller.GetBButtonPressed()) {
  //   t_servo.SetAngle(90);
  // }
  if (r_controller.GetXButton()) {
    Position1();
  }
  if (r_controller.GetBButton()) {
    Position2();
  }

  // Get the joystick input (-1 to 1)
    double speed1 = 0.1*r_controller.GetRawAxis(1);
    double speed2 = 0.1*r_controller.GetRawAxis(1);
    
    // Set the motor speed
    m_max12.Set(speed2);

  
  
  
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
