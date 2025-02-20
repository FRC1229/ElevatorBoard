// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "robot.h"
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <algorithm>
#include <units/time.h>

frc::Joystick stick(0);

// Define PID variable and initilize
//

double kP=0.03;
double kI=0.001;
double kD=0.00;
double autoAccelFactor;


// Creates a PIDController with gains kP, kI, and kD
frc::PIDController pid{kP, kI, kD};


using namespace rev::spark;
  // initialize motors
  
  SparkMax m_motor_11{11, SparkMax::MotorType::kBrushless};
  
  SparkMax m_motor_12{12, SparkMax::MotorType::kBrushless};
  
  rev::spark::SparkRelativeEncoder encoder(m_motor_11.GetEncoder());







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


double setpointPosition=0;
double encoderValue=encoder.GetPosition();
double manualSpeed=0;
double autoSpeed=0;
double elevatorPosition=(encoderValue/0.786)+2;

double lowSoftLimit=4;
double highSoftLimit=44;
double autoSpeedLimit=0.4;



//manual speed from stick, deadband of 0.2
if (stick.GetRawAxis(1)>0.2||stick.GetRawAxis(1)<-0.2){
   manualSpeed=-0.1*stick.GetRawAxis(1);
    }

else {
     manualSpeed=0; 
    }


//PID control move to position 1
if (stick.GetRawButton(1)){
  setpointPosition=10;
if(autoAccelFactor<1)
{
   autoAccelFactor=autoAccelFactor+0.02;
   
}
   autoSpeed=autoSpeedLimit*autoAccelFactor*pid.Calculate(elevatorPosition,setpointPosition);
   m_motor_11.Set(autoSpeed);
   m_motor_12.Set(autoSpeed);
   
}

   
    

 //PID control move to position 2
else if (stick.GetRawButton(2)){
  setpointPosition=30;

if(autoAccelFactor<1)
{
autoAccelFactor=autoAccelFactor+0.02;

}
   
   autoSpeed=autoSpeedLimit*autoAccelFactor*pid.Calculate(elevatorPosition,setpointPosition);
   m_motor_11.Set(autoSpeed);
   m_motor_12.Set(autoSpeed);
}
   
    

 //manual control
else {
 
 //manual control within soft limits
  if(elevatorPosition>lowSoftLimit&&elevatorPosition<highSoftLimit){
   m_motor_11.Set(manualSpeed);
   m_motor_12.Set(manualSpeed);
}

//manual control below low soft limit
if(elevatorPosition<lowSoftLimit&&manualSpeed>0){
  
   m_motor_11.Set(manualSpeed);
   m_motor_12.Set(manualSpeed);
}

//manual control above high soft limit
if(elevatorPosition>highSoftLimit&&manualSpeed<0){
  
   m_motor_11.Set(manualSpeed);
   m_motor_12.Set(manualSpeed);
}

if((elevatorPosition>highSoftLimit&&manualSpeed>0)||(elevatorPosition<lowSoftLimit&&manualSpeed<0)){
   m_motor_11.Set(0);
   m_motor_12.Set(0);
}
}

if (stick.GetRawButton(1)==0&&stick.GetRawButton(2)==0)
{
   autoAccelFactor=0;
}   

frc::SmartDashboard::PutNumber("Encoder Raw", encoderValue);
frc::SmartDashboard::PutNumber("Auto Speed", autoSpeed);
frc::SmartDashboard::PutNumber("Manual Speed", manualSpeed);
frc::SmartDashboard::PutNumber("Setpoint", setpointPosition);
frc::SmartDashboard::PutNumber("Actual Position", elevatorPosition);
frc::SmartDashboard::PutNumber("Accel Factor", autoAccelFactor);
frc::SmartDashboard::PutNumber("Speed Limit", autoSpeedLimit);





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
