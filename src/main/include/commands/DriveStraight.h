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
 * A command that will go straight at the specified speed
 */
class DriveStraight : public frc2::CommandHelper<frc2::PIDCommand, DriveStraight> {
 public:
  /**
   * Turns to robot to the specified angle.
   *
   * @param power              The motor power level to drive at
   * @param drive              The drive subsystem to use
   */
  DriveStraight(double power, DriveSubsystem* drive);

  bool IsFinished() override;
};
