/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimberHighClimb.h"

ClimberHighClimb::ClimberHighClimb(ClimberSubsystem* subsystem, frc::XboxController* controller) 
    : m_climber(subsystem), m_controller(controller)  {
  AddRequirements({subsystem});
}

void ClimberHighClimb::Initialize() {

}

void ClimberHighClimb::Execute() {
  m_climber->HighClimb();
}

void ClimberHighClimb::End(bool interrupted) {
 m_climber->Stop(); //should do this anyways with m_climber.SetDefaultCommand
}

bool ClimberHighClimb::IsFinished() {
  return false;
}