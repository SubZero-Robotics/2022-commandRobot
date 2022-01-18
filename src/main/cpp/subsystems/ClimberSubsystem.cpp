#include <frc/smartdashboard/Smartdashboard.h>

#include "subsystems/ClimberSubsystem.h"

#include "Constants.h" 

ClimberSubsystem::ClimberSubsystem() {
  // Implementation of subsystem constructor goes here.
  Winch.Set(ControlMode::PercentOutput, 0.0);
  Climber.Set(ControlMode::PercentOutput, 0.0);
}

// Methods

// Implementation of subsystem periodic method goes here.
// for example, publish encoder settings or motor currents to dashboard
void ClimberSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ClimberSubsystem::UpUp(){
  Winch.Set(ControlMode::PercentOutput, 0.35);//.35, .65
}

void ClimberSubsystem::DownDown(){
  Winch.Set(ControlMode::PercentOutput, -0.35);//-.35, .65
}

void ClimberSubsystem::Climb(){
  Climber.Set(ControlMode::PercentOutput, -0.8);//.5, .8
}

void ClimberSubsystem::Stop(){
  Climber.Set(ControlMode::PercentOutput, 0.0);
  Winch.Set(ControlMode::PercentOutput, 0.0);
}