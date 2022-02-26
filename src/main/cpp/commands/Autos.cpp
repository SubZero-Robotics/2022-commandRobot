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

LeftAuto::LeftAuto(DriveSubsystem* drive, CargoSubsystem * cargo) {
  AddCommands(
    frc2::ParallelRaceGroup( 
        DriveTwoBallUnoTrajectory(drive),     
        IntakeGrabBalls(cargo)),
      ShooterShoot(cargo, NULL).WithTimeout(2_s),
      frc2::ParallelRaceGroup( 
        DriveTwoBallDosTrajectory(drive),     
        IntakeGrabBalls(cargo)),
      ShooterShoot(cargo, NULL).WithTimeout(4_s),
// Next, try adding this second one.  Commented out for now in case there's a 
// sequencing problem
    DriveDistance(1.0_m,drive)

//      DriveStraight(-0.5, drive).WithTimeout(0.5_s), 
//      TurnToAngle(0_deg, drive).WithTimeout(5_s)
      //TurnToLimelight(drive).WithTimeout(2_s)
      /*// Drive back the specified time.  Positive power really is backwards
      DriveStraight(0.6, drive).WithTimeout(1_s),
      // aim with limelight: turn on light and turn to target at same time
      TurnToLimelight(drive).WithTimeout(2_s),
      // rev up shooter.  This is the simple, brute force way to shoot
      ShooterShoot(shooter,NULL).WithTimeout(1_s),
      // run indexer and shooter at the same time
      frc2::ParallelCommandGroup{ShooterShoot(shooter,NULL).WithTimeout(1_s),
                                 IndexerForward(indexer).WithTimeout(1_s)},
      // turn around 180_deg.  Timeout here is in case it doesn't settle in 2s
      TurnToAngle(180_deg, drive).WithTimeout(2_s),
      // drive forward one more second
      DriveStraight(-0.6, drive).WithTimeout(1_s)*/
);
}

RightAuto::RightAuto(DriveSubsystem* drive, CargoSubsystem * cargo) {
  AddCommands(
      // Drive back the specified time.  Positive power really is backwards
      DriveStraight(0.6, drive).WithTimeout(1_s),
      // aim with limelight: turn on light and turn to target at same time
      // also start to spin up shooter
      frc2::ParallelRaceGroup{ShooterShoot(cargo,NULL),
                              TurnToLimelight(drive).WithTimeout(2_s)},
      // Let's try a smarter shooting, where we check if we're at the right rpm and run the indexer
      frc2::ParallelCommandGroup{ShooterShoot(cargo,NULL).WithTimeout(1_s)},
      // turn around 180_deg.  Timeout here is in case it doesn't settle in 2s
      TurnToAngle(90_deg, drive).WithTimeout(2_s)
  );
}

/*NoAuto::NoAuto(DriveSubsystem* drive, CargoSubsystem* shooter) {
  AddCommands(

      //DriveStraight().WithTimeout()

  );

}*/