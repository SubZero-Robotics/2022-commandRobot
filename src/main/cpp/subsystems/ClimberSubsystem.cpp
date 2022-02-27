#include <frc/smartdashboard/Smartdashboard.h>

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
  if (abs(Xbox.GetRightY())>kDeadzone && Xbox.GetPOV()==180){
    RightArm.Set(Xbox.GetRightY()/1.5);
    LeftArm.Set(Xbox.GetRightY()/1.5);
  } else {
    RightArm.StopMotor();
    
    LeftArm.StopMotor(); 
  }

  if (Xbox.GetPOV()==0){
    MiddleArmAngle.Set(-Xbox.GetRightX()/3.5);
    MiddleArmOut.Set(Xbox.GetRightY()/1.5);
  } else {
    MiddleArmAngle.StopMotor();
    MiddleArmOut.StopMotor();
  }

}

void ClimberSubsystem::Up(){
  RightArm.Follow(LeftArm);
  LeftArm.Set(-1);
}

void ClimberSubsystem::Down(){
  RightArm.Follow(LeftArm);
  LeftArm.Set(1);
}

void ClimberSubsystem::Stop(){
  RightArm.StopMotor();
  LeftArm.StopMotor();
}