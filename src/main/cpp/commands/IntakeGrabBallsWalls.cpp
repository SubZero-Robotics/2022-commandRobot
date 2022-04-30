/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeGrabBallsWalls.h"

IntakeGrabBallsWalls::IntakeGrabBallsWalls(CargoSubsystem* subsystem) : m_cargo(subsystem) {
  AddRequirements({subsystem});
}

void IntakeGrabBallsWalls::Initialize() {

}

void IntakeGrabBallsWalls::Execute() {
  m_cargo->GrabBalls();
}

void IntakeGrabBallsWalls::End(bool interrupted) {
  m_cargo->Stop();
}

// this is a state, it lasts till it's cancelled
bool IntakeGrabBallsWalls::IsFinished() { return false; }