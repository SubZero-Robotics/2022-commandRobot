/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeAutoGrabBalls.h"

IntakeAutoGrabBalls::IntakeAutoGrabBalls(units::second_t durationOfMove, CargoSubsystem* subsystem) : m_length(durationOfMove), m_cargo(subsystem) {
  AddRequirements({subsystem});
}

void IntakeAutoGrabBalls::Initialize() {

}

void IntakeAutoGrabBalls::Execute() {
  m_cargo->AutoGrabBalls(m_length);
}

void IntakeAutoGrabBalls::End(bool interrupted) {
  m_cargo->Stop();
}

// this is a state, it lasts till it's cancelled
bool IntakeAutoGrabBalls::IsFinished() { return false; }