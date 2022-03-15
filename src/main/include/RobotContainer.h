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
#include "subsystems/CargoSubsystem.h"
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
  CargoSubsystem m_cargo;
  ClimberSubsystem m_climber;
  
  // The auto routines
 // add in whichever subsystems you use in this call list
  ThreeBallDownAuto m_threeballdownAuto{&m_drive, &m_cargo};
  FourBallFeedAuto m_fourballfeedAuto{&m_drive, &m_cargo};
  ThreeBallUpAuto m_threeballupAuto{&m_drive, &m_cargo};

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  // The controller
  frc::XboxController Xbox{0};

  // declarations for local functions
  void ConfigureButtonBindings();
};
