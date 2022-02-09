/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h> 
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

class DriveStrajectory
    : public frc2::CommandHelper<frc2::CommandBase, DriveStrajectory> {
 public:
  /**
   * Creates a new DriveStrajectory.
   *
   * takes no parameters: the trajectory is hardcoded in here
   * 
   */
  DriveStrajectory(DriveSubsystem* subsystem);

  void Initialize() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
};
