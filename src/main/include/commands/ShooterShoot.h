/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ShooterSubsystem.h"
 
/**
 * A simple command that runs ShooterSubsystem forwards.  Written
 * explicitly for pedagogical purposes.  Actual code should inline a command
 * this simple with InstantCommand.
 *
 * @see InstantCommand
 */
class ShooterShoot : public frc2::CommandHelper<frc2::CommandBase, ShooterShoot> {
 public:
  explicit ShooterShoot(ShooterSubsystem* subsystem, frc::XboxController* controller);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ShooterSubsystem* m_shooter;
  frc::XboxController* m_controller;
};
