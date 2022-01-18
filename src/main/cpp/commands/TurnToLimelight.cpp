/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnToLimeLight.h"

#include <frc/controller/PIDController.h>

#include <cmath>

// code based on this example
// https://github.com/wpilibsuite/allwpilib/tree/master/wpilibcExamples/src/main/cpp/examples/GyroDriveCommands
// The example uses the lambda in the constructor to do everything.  Ick.
// left the regular TurnToAngle that way, and trying to re-write this in a more clear way

TurnToLimelight::TurnToLimelight(DriveSubsystem* subsystem) : m_drive(subsystem) { 
        AddRequirements({subsystem});
}

void TurnToLimelight::Initialize() {
  //Turn on limelight
  m_drive->SelectLimelightPipeline(0);
}

void TurnToLimelight::Execute() {
  TurnToAngle(m_drive->GetLimelightTargetAngle(), m_drive);
}

void TurnToLimelight::End(bool interrupted) {
  // return limelight to camera mode
  m_drive->SelectLimelightPipeline(1);
  // stop drive
  m_drive->ArcadeDrive(0.0, 0.0);
  // reset controller, in case it gets re-used
  m_controller.Reset();
}

bool TurnToLimelight::IsFinished() { return m_controller.AtSetpoint(); }
