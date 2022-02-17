/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeTopIn.h"

IntakeTopIn::IntakeTopIn(IntakeSubsystem* subsystem) : m_intake(subsystem) {
  AddRequirements({subsystem});
}

void IntakeTopIn::Initialize() { m_intake->TopIn(); }

// this is a state, it lasts till it's cancelled
// although we could check if the piston is all the way in
bool IntakeTopIn::IsFinished() { return false; }
