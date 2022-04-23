/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeStop.h"

IntakeStop::IntakeStop(CargoSubsystem* subsystem) : m_cargo(subsystem) {
  AddRequirements({subsystem});
}

void IntakeStop::Initialize() { m_cargo->StopInt(); }

void IntakeStop::Execute() { m_cargo->Stop(); }

// this is a state, it lasts till it's cancelled
// It is also the default state, so needs to run forever
// although we could check if we hit an electic eye
bool IntakeStop::IsFinished() { return false; }
