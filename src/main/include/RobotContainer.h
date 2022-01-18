/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/Command.h> 
#include <frc/smartdashboard/SendableChooser.h>

#include <frc/XboxController.h>

#include "commands/Autos.h"
#include "commands/DefaultDrive.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/IndexerSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ClimberSubsystem.h"


/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // The robot's subsystems and commands are defined here...
  
  // The robot's subsystems
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  IndexerSubsystem m_indexer;
  ShooterSubsystem m_shooter;
  ClimberSubsystem m_climber;
  
  // The auto routines
  LeftAuto m_leftAuto{&m_drive, &m_shooter, &m_indexer};  // add in whichever subsystems you use in this call list
  RightAuto m_rightAuto{&m_drive, &m_shooter, &m_indexer};

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  // The controller
  frc::XboxController Xbox{0};

  // declarations for local functions
  void ConfigureButtonBindings();
};
