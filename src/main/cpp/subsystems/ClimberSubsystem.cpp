#include <frc/smartdashboard/Smartdashboard.h>

#include "subsystems/ClimberSubsystem.h"


ClimberSubsystem::ClimberSubsystem() {
  // Implementation of subsystem constructor goes here.
  //Winch.Set(ControlMode::PercentOutput, 0.0);
  //Climber.Set(ControlMode::PercentOutput, 0.0);
}

// Methods

// Implementation of subsystem periodic method goes here.
// for example, publish encoder settings or motor currents to dashboard
void ClimberSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ClimberSubsystem::Up(){
  RightArm.Set(-0.5);
  LeftArm.Set(-0.5);
}

void ClimberSubsystem::Down(){
  RightArm.Set(0.5);
  LeftArm.Set(0.5);
}

void ClimberSubsystem::Climb(){
  if (abs(Xbox.GetRightY())>kDeadzone){
  RightArm.Set(-Xbox.GetRightY()/2);
  LeftArm.Set(-Xbox.GetRightY()/2);
  } 
}

void ClimberSubsystem::Stop(){
  RightArm.StopMotor();
  LeftArm.StopMotor();
}