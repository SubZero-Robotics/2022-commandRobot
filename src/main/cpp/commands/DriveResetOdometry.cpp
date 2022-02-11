#include "commands/DriveResetOdometry.h"
#include <frc/geometry/Pose2d.h>

DriveResetOdometry::DriveResetOdometry(DriveSubsystem* subsystem, frc::XboxController* controller) 
    : m_drive(subsystem), m_controller(controller)  {
  AddRequirements({subsystem});
}

void DriveResetOdometry::Initialize() { 
  frc::Pose2d currentRobotPose;      // is also zeroed by default
  m_drive->ResetOdometry(currentRobotPose);
}

void DriveResetOdometry::Execute() {
  
}

void DriveResetOdometry::End(bool interrupted) {
  //should do this anyways with m_climber.SetDefaultCommand
}

bool DriveResetOdometry::IsFinished() {
  return false;
}