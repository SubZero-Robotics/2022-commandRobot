/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

#include "commands/DefaultDrive.h"
#include "commands/ExtendIntake.h"
#include "commands/RetractIntake.h"
#include "commands/SpitIntake.h"
#include "commands/IndexerForward.h"
#include "commands/IndexerBackward.h"
#include "commands/IndexerStop.h"
#include "commands/ShooterShoot.h"
#include "commands/ShooterStop.h"
#include "commands/ShooterUnjam.h"
#include "commands/TurnToLimelight.h"
#include "commands/TurnToAngle.h"


#include <utility>

#include "commands/ClimberUpUp.h"
#include "commands/ClimberDownDown.h"
#include "commands/ClimberClimb.h"
#include "commands/ClimberStop.h"


#include "Constants.h"

RobotContainer::RobotContainer() { 
  // Initialize all of your commands and subsystems here

  // Add commands to the autonomous command chooser
  m_chooser.SetDefaultOption("Left Auto", &m_leftAuto);
  m_chooser.AddOption("Right Auto", &m_rightAuto);

  // Put the chooser on the dashboard
  frc::Shuffleboard::GetTab("Autonomous").Add(m_chooser);

  // Configure the button bindings
  ConfigureButtonBindings(); 

// Set up default drive command.  Does this whenever the DriveSubSystem isn't doing anything else
  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive,
      [this] { return Xbox.GetLeftY(); },
      [this] { return Xbox.GetLeftX(); }));

// Set default intake command.  Does this when not doing something else
  m_intake.SetDefaultCommand(RetractIntake(&m_intake));

// Set default indexer command.  Does this when not doing something else
  m_indexer.SetDefaultCommand(IndexerStop(&m_indexer));

// Set default shooter command.  Does this when not doing something else
  m_shooter.SetDefaultCommand(ShooterStop(&m_shooter));

// Set default climber command.  Does this when not doing something else
  m_climber.SetDefaultCommand(ClimberStop(&m_climber));
}

// will need SelectCommand at some point:
// https://github.com/wpilibsuite/allwpilib/tree/master/wpilibcExamples/src/main/cpp/examples/SelectCommand

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  // Just driving isn't a button, it's the default mode for the DriveTrain 
  // subsystem, see above

  // Why use these new JoyStickButton commands?  Because they're the "command" way to do it
  
  //  .WhileHeld() keeps firing off the job while you hold the button, rescheduling it if it stops.
  //  .WhenHeld() only does it once
  //  both cancel the job when you let go of the button.
  // We have most of these commands go forever, to get canceled when the button is let off?
  // how to mess with them in auto, then?  Need end conditions, and Triggers that aren't
  // tied to buttons.  Answer: timeouts.
  // So, changed the .WhileHeld() to .WhenHeld()

  // Spin up shooter motor while pressed, and rumble controller if you're too close
  frc2::JoystickButton(&Xbox, Button::kB)
      .WhenHeld(ShooterShoot(&m_shooter, &Xbox));

  // move intake arm out and spin intake wheels while A is held down,
  // return arm and stop when you let go. (the default mode for Intake)
  frc2::JoystickButton(&Xbox, Button::kA)
      .WhenHeld(ExtendIntake(&m_intake));
// you can stack commands like this (below).  But in this case, RetractIntake is the default anyway
//      .WhenReleased(RetractIntake(&m_intake)); 

  // Run indexer forwards to suck in balls and send them to the shooter
  frc2::JoystickButton(&Xbox, Button::kX)
      .WhenHeld(IndexerForward(&m_indexer));

  // Climber deployment and winch: Y button plus right stick
  //frc2::JoystickButton(&Xbox, Axis::kRightY) 
      //.WhenHeld(ClimberUpUp(&m_climber, &Xbox));

  /*if (Xbox.GetY(frc::GenericHID::kRightHand)>kDeadzone){
    ClimberUpUp(&m_climber, &Xbox);
  } else if (Xbox.GetY(frc::GenericHID::kRightHand)>-kDeadzone) { //////////////////
    ClimberDownDown(&m_climber, &Xbox);
  }*/

  frc2::JoystickButton(&Xbox, Button::kBumperLeft)
      .WhenHeld(ClimberUpUp(&m_climber, &Xbox));

  frc2::JoystickButton(&Xbox, Button::kStickRight)
      .WhenHeld(ClimberDownDown(&m_climber, &Xbox));
  
  frc2::JoystickButton(&Xbox, Button::kY)
      .WhenHeld(ClimberClimb(&m_climber, &Xbox)); 
  
  // this logic will need Camden's explanation to implement
  // limelight aiming.  
  frc2::JoystickButton(&Xbox, Button::kBumperRight)
      .WhenHeld(TurnToLimelight(&m_drive).WithTimeout(2_s));

  // unjam things
  // Run indexer and shooter backwards.  ParallelCommandGroup finishes when all of the commands finish
  // Could change this to ParallelRaceGroup if the indexer commands exit on laserbreaks
  frc2::JoystickButton(&Xbox, Button::kStart)
      .WhenHeld(frc2::ParallelCommandGroup{IndexerBackward(&m_indexer),
                                            ShooterUnjam(&m_shooter)
                                            });
  // Run indexer and shooter backwards AND burp them out the intake
  frc2::JoystickButton(&Xbox, Button::kBack)
      .WhenHeld(frc2::ParallelCommandGroup{IndexerForward(&m_indexer),
                                            SpitIntake(&m_intake),
                                            ShooterUnjam(&m_shooter)
                                            });
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Runs the chosen command in autonomous
  return m_chooser.GetSelected();
}
