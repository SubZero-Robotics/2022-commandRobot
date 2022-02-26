/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimberHighClimbAngle.h"

ClimberHighClimbAngle::ClimberHighClimbAngle(ClimberSubsystem* subsystem, frc::XboxController* controller) 
    : m_climber(subsystem), m_controller(controller)  {
  AddRequirements({subsystem});
}

void ClimberHighClimbAngle::Initialize() {

}

void ClimberHighClimbAngle::Execute() {
  m_climber->HighClimbAngle();
}

void ClimberHighClimbAngle::End(bool interrupted) {
 m_climber->Stop(); //should do this anyways with m_climber.SetDefaultCommand
}

bool ClimberHighClimbAngle::IsFinished() {
  return false;
}