/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveDistance.h"

#include <cmath> 

/* right now this command doesn't do anything.  It needs an Execute() method to 
   check the encoders to see if it's gone the requested distance.

   Once we've got the Ramsetes stuff all working, this should create a pose with 
   the target location and use a RamsetesCommand to go to that */

DriveDistance::DriveDistance(double feet, double speed,
                             DriveSubsystem* subsystem)
    : m_drive(subsystem), m_distance(feet), m_speed(speed) {
  AddRequirements({subsystem});
}

void DriveDistance::Initialize() {
  m_drive->ResetEncoders();
  m_drive->ArcadeDrive(m_speed, 0);
}

void DriveDistance::End(bool interrupted) { m_drive->ArcadeDrive(0, 0); }

bool DriveDistance::IsFinished() {  
  return false;
}
