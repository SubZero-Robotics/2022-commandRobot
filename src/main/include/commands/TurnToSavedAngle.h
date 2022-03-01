/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>

#include "subsystems/DriveSubsystem.h"

/**
 * A command that will turn the robot to the TargetAngle saved by the drive system
 */
class TurnToSavedAngle : public frc2::CommandHelper<frc2::PIDCommand, TurnToSavedAngle> {
 public:
  /**
   * Turns to robot to the specified angle.
   *
   * @param drive              The drive subsystem to use
   */
  TurnToSavedAngle(DriveSubsystem* drive);

  bool IsFinished() override;
};
