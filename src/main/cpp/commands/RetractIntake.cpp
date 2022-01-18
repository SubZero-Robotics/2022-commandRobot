/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RetractIntake.h"

RetractIntake::RetractIntake(IntakeSubsystem* subsystem) : m_intake(subsystem) {
  AddRequirements({subsystem});
}

void RetractIntake::Initialize() { m_intake->Retract(); }

// this is a state, it lasts till it's cancelled
// It is also the default command, so needs to run forever
// although we could check if the piston is all the way in
bool RetractIntake::IsFinished() { return false; }
