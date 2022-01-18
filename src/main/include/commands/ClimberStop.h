/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ClimberSubsystem.h"

/**
 * A simple command that runs ClimberSubsystem forwards.  Written
 * explicitly for pedagogical purposes.  Actual code should inline a command
 * this simple with InstantCommand.
 *
 * @see InstantCommand
 */
class ClimberStop : public frc2::CommandHelper<frc2::CommandBase, ClimberStop> {
 public:
  explicit ClimberStop(ClimberSubsystem* subsystem);

  void Initialize() override;

  bool IsFinished() override;

 private:
  ClimberSubsystem* m_climber;
};