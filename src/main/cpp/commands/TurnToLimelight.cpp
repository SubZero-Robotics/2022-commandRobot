/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TurnToLimeLight.h"
#include <frc2/command/WaitCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

// This doesn't really turn to limelight.  
// It turns on the limelight, waits till it gets a lock on the target, then
// sets the target location and finishes.

TurnToLimelight::TurnToLimelight(DriveSubsystem* subsystem,
                           std::function<double()> forward,
                           std::function<double()> rotation)
    : m_drive{subsystem}, m_forward{forward}, m_rotation{rotation} {
  AddRequirements({subsystem});
}

void TurnToLimelight::Initialize() {
  //Turn on limelight
  //m_drive->SelectLimelightPipeline(0);
}

void TurnToLimelight::Execute() {
  //turntothis = (m_drive->GetGyroAngle() + nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0));
  //frc::SmartDashboard::PutNumber("turntothis", turntothis);
  //TurnToAngle((units::degree_t)turntothis, m_drive);
    //finished = true;

  
  // Apply stick deadzone 
  double XboxX = m_rotation();
  if(abs(XboxX) < kDeadzone/2) XboxX = 0.0;
  double XboxY = m_forward();
  if(abs(XboxY) < kDeadzone/2) XboxY = 0.0;

  // drive it
  m_drive->ArcadeDrive(XboxY, XboxX);
  }

bool TurnToLimelight::IsFinished() { return finished; }

void TurnToLimelight::End(bool interrupted) {
  // return limelight to camera mode
  m_drive->SelectLimelightPipeline(1);
}
