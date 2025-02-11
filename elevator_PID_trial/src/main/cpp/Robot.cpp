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
#include <rev/SparkAbsoluteEncoder.h>
#include <algorithm>
#include <units/voltage.h>
#include <units/length.h>

frc::Joystick stick(0);

// Define PID variable and initilize

double kP=0.02;
double kI=0.0;
double kD=0.00;
units::volt_t kS = 4_V;
units::volt_t kG = 2_V;
units::volt_t kV = 2_V/0.5_mps;
units::volt_t kA = 2_V/0.5_mps_sq;



// Creates a PIDController with gains kP, kI, and kD
frc::ProfiledPIDController<units::meters> m_controller(
   kP, kI, kD, 
   frc::TrapezoidProfile<units::meters>::Constraints{0.3_mps, 0.3_mps_sq});
frc::ElevatorFeedforward m_feedforward(kS, kG, kV, kA);

frc::TrapezoidProfile<units::meters>::State goal1;
frc::TrapezoidProfile<units::meters>::State goal;

using namespace rev::spark;
  // initialize motors
  
  SparkMax m_motor_11{11, SparkMax::MotorType::kBrushless};
  
  //SparkMax m_motor_12{12, SparkMax::MotorType::kBrushless};
  
  //rev::spark::SparkRelativeEncoder encoder(m_motor_11.GetRel);
  rev::spark::SparkRelativeEncoder encoder(m_motor_11.GetEncoder());







Robot::Robot() {
  
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
 
m_motor_11.SetInverted(true);
//m_motor_12.SetInverted(false);                   





};

void Robot::TeleopPeriodic() {


double setpointPosition=0;
double encoderValue=encoder.GetPosition();
double manualSpeed=0;
double autoSpeed=0;
units::meter_t elevatorPosition = units::meter_t((encoderValue/0.786)+2);
units::volt_t pidOutput = units::volt_t{m_controller.Calculate(elevatorPosition)};
units::volt_t feedforwardOutput = m_feedforward.Calculate(goal1.position, goal1.velocity);

double lowSoftLimit=4;
double highSoftLimit=44;


//manual speed from stick, deadband of 0.2
// if (stick.GetRawAxis(1)>0.2||stick.GetRawAxis(1)<-0.2){
//    manualSpeed=-0.1*stick.GetRawAxis(1);
//     }

// else {
//      manualSpeed=0; 
//     }


//PID control move to position 1
// if (stick.GetRawButton(1)){
//   units::meter_t setpointPosition= 1_m;
//    autoSpeed=0.7*pid.Calculate(elevatorPosition, setpointPosition);
//    m_motor_11.SetVoltage(feedforwardOutput + pidOutput);
//    //m_motor_12.Set(autoSpeed);
//     }
if (stick.GetRawButton(1)){
  goal = {1_m, 0_mps};
  
   //m_motor_12.Set(autoSpeed);
    }

 //PID control move to position 2
else if (stick.GetRawButton(2)){
   goal = {0.5_m, 0_mps};
   //m_motor_12.Set(autoSpeed);
    }  

 //manual control
else {

 
 //manual control within soft limits
//   if(elevatorPosition>lowSoftLimit&&elevatorPosition<highSoftLimit){
//    m_motor_11.Set(manualSpeed);
//    //m_motor_12.Set(manualSpeed);
// }

// //manual control below low soft limit
// if(elevatorPosition<lowSoftLimit&&manualSpeed>0){
  
//    m_motor_11.Set(manualSpeed);
//    //m_motor_12.Set(manualSpeed);
// }

// //manual control above high soft limit
// if(elevatorPosition>highSoftLimit&&manualSpeed<0){
  
//    m_motor_11.Set(manualSpeed);
//    //m_motor_12.Set(manualSpeed);
// }

// if((elevatorPosition>highSoftLimit&&manualSpeed>0)||(elevatorPosition<lowSoftLimit&&manualSpeed<0)){
//    m_motor_11.Set(0);
//    //m_motor_12.Set(0);
// }
}

m_controller.SetGoal(goal);
m_motor_11.SetVoltage(
        units::volt_t{
            m_controller.Calculate(units::meter_t{encoder.GetPosition()*0.786 + 2})} +
        m_feedforward.Calculate(m_controller.GetSetpoint().velocity));

     

frc::SmartDashboard::PutNumber("Encoder Raw", encoderValue);
frc::SmartDashboard::PutNumber("Auto Speed", autoSpeed);
frc::SmartDashboard::PutNumber("Manual Speed", manualSpeed);
frc::SmartDashboard::PutNumber("Setpoint", setpointPosition);
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
