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

  if (abs(Xbox.GetRightY())>kDeadzone && Xbox.GetPOV()==0){
    MiddleArmAngle.Set(Xbox.GetRightY()/1.5);
  } else {
    MiddleArmAngle.StopMotor();
  }
  
  if (abs(Xbox.GetRightX())>kDeadzone && Xbox.GetPOV()==0){
    MiddleArmOut.Set(Xbox.GetRightX()/1.5);
  } else {
    MiddleArmOut.StopMotor();
  }
  

}

void ClimberSubsystem::Up(){
  RightArm.Set(-0.5);
  LeftArm.Set(-0.5);
}

void ClimberSubsystem::Down(){
  RightArm.Set(0.5);
  LeftArm.Set(0.5);
}

void ClimberSubsystem::HighClimb(){
  MiddleArmOut.Set(0.5);
}

void ClimberSubsystem::HighClimbAngle(){
  MiddleArmAngle.Set(0.5);
}

void ClimberSubsystem::Stop(){
  RightArm.StopMotor();
  LeftArm.StopMotor();
}