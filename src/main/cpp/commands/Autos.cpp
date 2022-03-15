/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Autos.h"

// Can "decorate" commands.  eg:
// command.WithTimeout(1_s)  cancels the command after the time has elapsed
// Will be interrupted if m_limitSwitch.get() returns true.  eg, electric eyes and ball intake
// command.InterruptOn([&m_limitSwitch] { return m_limitSwitch.Get(); });

/* LeftAuto::LeftAuto(DriveSubsystem* drive, CargoSubsystem * cargo) {
  AddCommands(
    frc2::ParallelRaceGroup( 
        DriveTwoBallUnoTrajectory(drive),     
        IntakeGrabBalls(cargo)),
      IntakeAllOut(cargo).WithTimeout(0.1_s),
      ShooterAutoShoot(cargo, NULL).WithTimeout(3_s),
      frc2::ParallelRaceGroup( 
        DriveTwoBallDosTrajectory(drive),     
        IntakeGrabBalls(cargo)),
      IntakeAllOut(cargo).WithTimeout(0.1_s),
      ShooterAutoShoot(cargo, NULL).WithTimeout(4_s),
  );
} */

StraightBackAuto::StraightBackAuto(DriveSubsystem* drive, CargoSubsystem* cargo) {
  AddCommands(
    StraightBackRun(drive, cargo)
  );
}


TwoBallUpAuto::TwoBallUpAuto(DriveSubsystem* drive, CargoSubsystem* cargo) {
  AddCommands(
    TwoBallUpRun(drive, cargo)
  );
}

ThreeBallDownAuto::ThreeBallDownAuto(DriveSubsystem* drive, CargoSubsystem* cargo) {
  AddCommands(
    ThreeBallDownRun(drive, cargo)
  );
}

FourBallFeedAuto::FourBallFeedAuto(DriveSubsystem* drive, CargoSubsystem* cargo) {
  AddCommands(
    FourBallFeedRun(drive, cargo)
  );
}

ThreeBallUpAuto::ThreeBallUpAuto(DriveSubsystem* drive, CargoSubsystem* cargo) {
  AddCommands(
    ThreeBallUpRun(drive, cargo)
  );
}
