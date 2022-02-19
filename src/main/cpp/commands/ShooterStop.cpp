/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShooterStop.h"

ShooterStop::ShooterStop(CargoSubsystem* subsystem) : m_cargo(subsystem) {
  AddRequirements({subsystem});
}

void ShooterStop::Initialize() { m_cargo->Stop(); }

// this is a state, it lasts till it's cancelled
// although we could check if we are at the right RPM
bool ShooterStop::IsFinished() { return false; }
