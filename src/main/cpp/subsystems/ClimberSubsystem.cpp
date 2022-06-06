#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/ClimberSubsystem.h"


ClimberSubsystem::ClimberSubsystem() {
  // Implementation of subsystem constructor goes here. (initial state)
  RightArm.StopMotor();
  LeftArm.StopMotor();
}

// Methods

// Implementation of subsystem periodic method goes here.
// for example, publish encoder settings or motor currents to dashboard
void ClimberSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void ClimberSubsystem::Up(){
  RightArm.Follow(LeftArm, true);
  LeftArm.Set(1);
}

void ClimberSubsystem::Down(){
  RightArm.Follow(LeftArm, true);
  LeftArm.Set(-1);
}

void ClimberSubsystem::Stop(){
  RightArm.StopMotor();
  LeftArm.StopMotor();
}