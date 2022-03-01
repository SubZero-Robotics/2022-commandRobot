/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnToLimeLight.h"

#include <cmath>

// This doesn't really turn to limelight.  
// It turns on the limelight, waits till it gets a lock on the target, then
// sets the target location and finishes.

TurnToLimelight::TurnToLimelight(DriveSubsystem* subsystem) : m_drive(subsystem) { 
        AddRequirements({subsystem});
}

void TurnToLimelight::Initialize() {
  //Turn on limelight
  m_drive->SelectLimelightPipeline(0);
}

void TurnToLimelight::Execute() {
  if (m_drive->GetLimelightTargetValid()) {
    m_drive->SetTargetAngle(m_drive->GetLimelightTargetAngle());
    finished = true;
  }
}

bool TurnToLimelight::IsFinished() { return finished; }

void TurnToLimelight::End(bool interrupted) {
  // return limelight to camera mode
  m_drive->SelectLimelightPipeline(1);
}
