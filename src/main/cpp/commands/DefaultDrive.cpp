/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <cmath>

#include "commands/DefaultDrive.h"
#include "Constants.h"

DefaultDrive::DefaultDrive(DriveSubsystem* subsystem,
                           std::function<double()> forward,
                           std::function<double()> rotation)
    : m_drive{subsystem}, m_forward{forward}, m_rotation{rotation} {
  AddRequirements({subsystem});
}

void DefaultDrive::Execute() {

  // Apply stick deadzone 
  double XboxX = m_rotation();
  if(abs(XboxX) < kDeadzone) XboxX = 0.0;
  double XboxY = m_forward();
  if(abs(XboxY) < kDeadzone) XboxY = 0.0;

  // drive it
  m_drive->ArcadeDrive(XboxY, XboxX);
}

// no end condition, this is the fallback state