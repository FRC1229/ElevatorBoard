// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "robot.h"
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
//#include <rev/SparkAbsoluteEncoder.h>
#include <algorithm>
#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>

frc::Joystick stick(0);

// Define PID variable and initilize

double kP=0.02;
double kI=0.0;
double kD=0.00;
units::volt_t kS = 0_V;
units::volt_t kG = 0_V;
auto kV = 2_V/(0.5_mps);
auto kA = 2_V/(0.5_mps_sq);
double manualSpeed;


// Creates a PIDController with gains kP, kI, and kD
frc::ProfiledPIDController<units::meters> m_controller(
   kP, kI, kD, 
   frc::TrapezoidProfile<units::meters>::Constraints{0.4_mps, 0.4_mps_sq});
frc::ElevatorFeedforward m_feedforward(kS, kG, kV, kA);
frc::TrapezoidProfile<units::meters>::State goal;

using namespace rev::spark;
  // initialize motors
  
  SparkMax m_motor_11{11, SparkMax::MotorType::kBrushless};
  
  SparkMax m_motor_12{12, SparkMax::MotorType::kBrushless};
  
  rev::spark::SparkRelativeEncoder encoder11(m_motor_11.GetEncoder());
  rev::spark::SparkRelativeEncoder encoder12(m_motor_12.GetEncoder());
  




Robot::Robot() {
  
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
 
m_motor_11.SetInverted(true);
m_motor_12.SetInverted(false);                   





};

void Robot::TeleopPeriodic() {




double lowSoftLimit=0.1;
double highSoftLimit=2.0;

double circumference = 0.025; // Pre-calculated circumference in meters
double revolutions11 = encoder11.GetPosition(); // Get revolutions from encoder
double revolutions12 = encoder12.GetPosition(); // Get revolutions from encoder
double distanceInMeters11 = revolutions11 * circumference; // Convert to meters
double distanceInMeters12 = revolutions12 * circumference; // Convert to meters


//manual speed from stick, deadband of 0.2
if (stick.GetRawAxis(1)>0.2||stick.GetRawAxis(1)<-0.2){
    manualSpeed=-0.1*stick.GetRawAxis(1);
     }

 else {
      manualSpeed=0; 
     }


//PID control move to position 1
// if (stick.GetRawButton(1)){
//   units::meter_t setpointPosition= 1_m;
//    autoSpeed=0.7*pid.Calculate(elevatorPosition, setpointPosition);
//    m_motor_11.SetVoltage(feedforwardOutput + pidOutput);
//    //m_motor_12.Set(autoSpeed);
//     }
if (stick.GetRawButton(1)){
  goal = {1_m, 0_mps};
    }

 //PID control move to position 2
else if (stick.GetRawButton(2)){
   goal = {0.5_m, 0_mps};
    }  







 //manual control


 
 //manual control within soft limits
 else  if(distanceInMeters11>lowSoftLimit&&distanceInMeters11<highSoftLimit){
    m_motor_11.Set(manualSpeed);
    //m_motor_12.Set(manualSpeed);
 }

// //manual control below low soft limit
else if(distanceInMeters11<lowSoftLimit&&manualSpeed>0){
  
    m_motor_11.Set(manualSpeed);
    //m_motor_12.Set(manualSpeed);
 }

// //manual control above high soft limit
 else if(distanceInMeters11>highSoftLimit&&manualSpeed<0){
  
    m_motor_11.Set(manualSpeed);
    //m_motor_12.Set(manualSpeed);
 }

 else if((distanceInMeters11>highSoftLimit&&manualSpeed>0)||(distanceInMeters11<lowSoftLimit&&manualSpeed<0)){
   m_motor_11.Set(0);
    //m_motor_12.Set(0);
 }











m_controller.SetGoal(goal);
units::meter_t e11Position = units::meter_t{distanceInMeters11};
units::meter_t e12Position = units::meter_t{distanceInMeters12};
units::volt_t pidCalc11 = units::volt_t{m_controller.Calculate(units::meter_t(e11Position))};
units::volt_t pidCalc12 = units::volt_t{m_controller.Calculate(units::meter_t(e12Position))};
units::volt_t feedForwardCalc = m_feedforward.Calculate(m_controller.GetSetpoint().velocity);
m_motor_11.SetVoltage(pidCalc11 + feedForwardCalc);
m_motor_12.SetVoltage(pidCalc12 + feedForwardCalc);



     

frc::SmartDashboard::PutNumber("Encoder 11 Raw", revolutions11);
frc::SmartDashboard::PutNumber("Encoder 12 Raw", revolutions12);
frc::SmartDashboard::PutNumber("Distance for encoder 11", distanceInMeters11);
frc::SmartDashboard::PutNumber("Distance for encoder 12", distanceInMeters12);
 frc::SmartDashboard::PutNumber("Manual Speed", manualSpeed);
// frc::SmartDashboard::PutNumber("Setpoint", setpointPosition);
//frc::SmartDashboard::PutNumber("Actual Position", elevatorPosition);




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
