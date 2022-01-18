/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IndexerSubsystem.h"

/**
 * A simple command that runs IndexerSubsystem forwards.  Written
 * explicitly for pedagogical purposes.  Actual code should inline a command
 * this simple with InstantCommand.
 *
 * @see InstantCommand
 */
class IndexerForwardCheckRPM : public frc2::CommandHelper<frc2::CommandBase, IndexerForwardCheckRPM> {
 public:
  explicit IndexerForwardCheckRPM(IndexerSubsystem* subsystem);

  void Initialize() override;

  void Execute() override;

  bool IsFinished() override;

  void End(bool interrupted) override;

 private:
  IndexerSubsystem* m_indexer;
};
