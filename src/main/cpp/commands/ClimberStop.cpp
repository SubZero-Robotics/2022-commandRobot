/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimberStop.h"

ClimberStop::ClimberStop(ClimberSubsystem* subsystem) : m_climber(subsystem) {
  AddRequirements({subsystem});
}

void ClimberStop::Initialize() { m_climber->Stop(); }

// this is a state, it lasts till it's cancelled
bool ClimberStop::IsFinished() { return false; }