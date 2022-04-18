/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "commands/DriveStraight.h"

#include "commands/autos/ThreeBallDown.h"
#include "commands/autos/FourBallFeed.h"
#include "commands/autos/ThreeBallUp.h"
#include "commands/autos/TwoBallUp.h"
#include "commands/autos/StraightBack.h"

#include "commands/IntakeGrabBalls.h"
#include "commands/IntakeAllOut.h"

#include "commands/ShooterShoot.h"
#include "commands/ShooterAutoShoot.h"
#include "commands/ShooterLowShoot.h"

#include "commands/DriveDistance.h"
#include "Constants.h"
#include <pathplanner/lib/PathPlanner.h>
#include "subsystems/DriveSubsystem.h"

/**
 * An auto for jamming everything into one file. Like RobotContainer.cpp
 */
class StraightBackAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, StraightBackAuto> {
 public:
  /**
   * Creates a new LeftAuto.
   *
   * @param drive The drive subsystem this command will run on
   */
  StraightBackAuto(DriveSubsystem* drive, CargoSubsystem* cargo);
};

/**
 * An auto for jamming everything into one file. Like RobotContainer.cpp
 */
class TwoBallUpAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, TwoBallUpAuto> {
 public:
  /**
   * Creates a new LeftAuto.
   *
   * @param drive The drive subsystem this command will run on
   */
  TwoBallUpAuto(DriveSubsystem* drive, CargoSubsystem* cargo);
};

/**
 * An auto for jamming everything into one file. Like RobotContainer.cpp
 */
class ThreeBallDownAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, ThreeBallDownAuto> {
 public:
  /**
   * Creates a new LeftAuto.
   *
   * @param drive The drive subsystem this command will run on
   */
  ThreeBallDownAuto(DriveSubsystem* drive, CargoSubsystem* cargo);
};

/**
 * Four ball feed auto by jamming everything in one file. Thanks Kaiden!
 */
class FourBallFeedAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, FourBallFeedAuto> {
 public:
  /**
   * Creates a new FourBallFeedAuto.
   *
   * @param drive The drive subsystem this command will run on
   */
  FourBallFeedAuto(DriveSubsystem* drive, CargoSubsystem* cargo);
};

/**
 * Three ball top auto by jamming everything in one file.
 */
class ThreeBallUpAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, ThreeBallUpAuto> {
 public:
  /**
   * Creates a new ThreeBallUpAuto.
   *
   * @param drive The drive subsystem this command will run on
   */
  ThreeBallUpAuto(DriveSubsystem* drive, CargoSubsystem* cargo);
};
