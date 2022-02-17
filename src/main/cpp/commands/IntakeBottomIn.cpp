/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeBottomIn.h"

IntakeBottomIn::IntakeBottomIn(IntakeSubsystem* subsystem) : m_intake(subsystem) {
  AddRequirements({subsystem});
}

void IntakeBottomIn::Initialize() { m_intake->BottomIn(); }

// this is a state, it lasts till it's cancelled
// although we could check if the piston is all the way in
bool IntakeBottomIn::IsFinished() { return false; }
