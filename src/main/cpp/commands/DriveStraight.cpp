/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveStraight.h"

#include <frc/controller/PIDController.h> 

DriveStraight::DriveStraight(double power, DriveSubsystem* drive)
    : CommandHelper(frc2::PIDController(kStabilizationP, kStabilizationI, kStabilizationD),
                    // Close loop on turn rate
                    [drive] { return drive->GetTurnRate(); },
                    // setpoint is not turning
                    0.0,
                    // Pipe output to turn robot while still going at "power"
                    [drive,power](double output) { drive->ArcadeDrive(power, output); },
                    // Require the drive
                    {drive}) {
  // we have no tolerance to set, since this keeps going till we cancel it

  AddRequirements({drive});
}

// keep going till the command is canceled
bool DriveStraight::IsFinished() { return false; }
