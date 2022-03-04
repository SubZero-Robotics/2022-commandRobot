/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "commands/TurnToAngle.h"
#include "subsystems/DriveSubsystem.h"

/**
 * A command that will turn the robot to the limelight's target
 */
class TurnToLimelight : public frc2::CommandHelper<frc2::CommandBase, TurnToLimelight> {
 public:
  /**
   * Turns to robot to where the limelight is pointing
   *
   * @param drive              The drive subsystem to use
   */
  explicit TurnToLimelight(DriveSubsystem* subsystem, std::function<double()> forward,
               std::function<double()> rotation);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    DriveSubsystem* m_drive;
    bool finished = false;
    double turntothis = 0.0;

  std::function<double()> m_forward;
  std::function<double()> m_rotation;
};
