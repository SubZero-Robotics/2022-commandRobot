/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/LimelightTimedCopy.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

LimelightTimedCopy::LimelightTimedCopy(DriveSubsystem* subsystem,
                           std::function<double()> forward,
                           std::function<double()> rotation)
    : m_drive{subsystem}, m_forward{forward}, m_rotation{rotation} {
  AddRequirements({subsystem});
}

void LimelightTimedCopy::Initialize() {
  //Turn on limelight
  m_drive->SelectLimelightPipeline(0);
}

void LimelightTimedCopy::Execute() {
   // Apply stick deadzone 
  double XboxX = m_rotation();
  if(abs(XboxX) < kDeadzone/2) XboxX = 0.0;
  double XboxY = m_forward();
  if(abs(XboxY) < kDeadzone/2) XboxY = 0.0;

  // drive it
  m_drive->LimelightTimedCopy(XboxY, XboxX);
  }

bool LimelightTimedCopy::IsFinished() { return finished; }

void LimelightTimedCopy::End(bool interrupted) {
  // return limelight to camera mode
  m_drive->SelectLimelightPipeline(1);
}
