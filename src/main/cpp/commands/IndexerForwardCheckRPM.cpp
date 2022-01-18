/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IndexerForwardCheckRPM.h"

IndexerForwardCheckRPM::IndexerForwardCheckRPM(IndexerSubsystem* subsystem) : m_indexer(subsystem) {
  AddRequirements({subsystem});
}

void IndexerForwardCheckRPM::Initialize() { }

// This command needs to do something each loop, so Execute not Initialize
void IndexerForwardCheckRPM::Execute() { m_indexer->ForwardCheckRPM(); }

// we can explicitly stop the indexer here.
// We don't for many of the other motor related things, because the default
// command for the subsystem is to stop stuff.
// could/should belt and suspenders this everywhere, like this?
void IndexerForwardCheckRPM::End(bool interrupted) { m_indexer->Stop(); }

// this is a state, it lasts till it's cancelled
// although we could check if we hit an electic eye, count balls or something
bool IndexerForwardCheckRPM::IsFinished() { return false; }
