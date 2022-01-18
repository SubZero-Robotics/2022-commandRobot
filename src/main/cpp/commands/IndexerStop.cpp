/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IndexerStop.h"

IndexerStop::IndexerStop(IndexerSubsystem* subsystem) : m_indexer(subsystem) {
  AddRequirements({subsystem});
}

void IndexerStop::Initialize() { m_indexer->Stop(); }

// this is a state, it lasts till it's cancelled
// It is also the default state, so needs to run forever
// although we could check if we hit an electic eye
bool IndexerStop::IsFinished() { return false; }
