/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IndexerBackward.h"

IndexerBackward::IndexerBackward(IndexerSubsystem* subsystem) : m_indexer(subsystem) {
  AddRequirements({subsystem});
}

void IndexerBackward::Initialize() { m_indexer->Backward(); }

// this is a state, it lasts till it's cancelled
// although we could check if we hit an electic eye
bool IndexerBackward::IsFinished() { return false; }
