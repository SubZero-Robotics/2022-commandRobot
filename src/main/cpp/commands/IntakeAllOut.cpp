/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeAllOut.h"

IntakeAllOut::IntakeAllOut(CargoSubsystem* subsystem) : m_cargo(subsystem) {
  AddRequirements({subsystem});
}

void IntakeAllOut::Initialize() { m_cargo->AllOut(); }

// this is a state, it lasts till it's cancelled
// although we could check if the piston is all the way in
bool IntakeAllOut::IsFinished() { return false; }
