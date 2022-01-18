/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShooterShoot.h"
#include <frc/smartdashboard/Smartdashboard.h>

ShooterShoot::ShooterShoot(ShooterSubsystem* subsystem, frc::XboxController* controller) 
    : m_shooter(subsystem), m_controller(controller)  {
  AddRequirements({subsystem});
}

void ShooterShoot::Initialize() { }
 
void ShooterShoot::Execute() {
  // tell shooter to get to set rpm
  m_shooter->Shoot();

  // Auto commands pass in NULL for the controller as there's no need to rumble
  if (m_controller != NULL) {
    // get distance from network tables to prevent a DriveSubsystem dependancy
    double distance = frc::SmartDashboard::GetNumber("Distance", 10000.0); // a lot of inches, by default
    // if we're too close, warn driver
    if (distance < kRumbleDistance)
      m_controller->SetRumble(frc::GenericHID::RumbleType::kLeftRumble,1.0);
    else
      m_controller->SetRumble(frc::GenericHID::RumbleType::kLeftRumble,0.0);  
  }
}

// stop rumbling when the command finishes, stop shooter (redundant with default command. hopefully)
void ShooterShoot::End(bool interrupted) {
  m_controller->SetRumble(frc::GenericHID::RumbleType::kLeftRumble,0.0);  
  m_shooter->Stop();
}

// this is a state, it lasts till it's cancelled
// although we could check if we are at the right RPM
bool ShooterShoot::IsFinished() { return false; }
