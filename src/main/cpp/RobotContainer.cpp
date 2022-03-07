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

#include "commands/IntakeAutomatic.h"
#include "commands/IntakeGrabBalls.h"
#include "commands/IntakeBottomIn.h"
#include "commands/IntakeAllOut.h"
#include "commands/IntakeTopIn.h"
#include "commands/IntakeTopOut.h"
#include "commands/IntakeStop.h"

#include "commands/ShooterLowShoot.h"
#include "commands/ShooterShoot.h"
#include "commands/ShooterAutoShoot.h"
#include "commands/ShooterStop.h"
#include "commands/ShooterUnjam.h"

#include "commands/ClimberUp.h"
#include "commands/ClimberDown.h"
#include "commands/ClimberStop.h"

#include "commands/DefaultDrive.h"
#include "commands/TurnToLimelight.h"
#include "commands/TurnToAngle.h"
#include "commands/TurnToSavedAngle.h"
#include "commands/DriveResetOdometry.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <utility>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

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
      [this] { return Xbox.GetLeftY()*0.8; },
      [this] { return Xbox.GetLeftX()*0.8; }));

// Set default intake, shooter, and indexer command.  Does this when not doing something else
  m_cargo.SetDefaultCommand(IntakeStop(&m_cargo));

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

  // Spin up shooter motor for low while pressed
  frc2::JoystickButton(&Xbox, Button::kB)
      .WhenHeld(frc2::SequentialCommandGroup(IntakeAllOut(&m_cargo).WithTimeout(0.05_s),
                                            ShooterLowShoot(&m_cargo, &Xbox)));

  //Spin up shooter motor for high while pressed, and rumble controller if you're too close
  frc2::JoystickButton(&Xbox, Button::kY)
      .WhenHeld(frc2::SequentialCommandGroup(IntakeAllOut(&m_cargo).WithTimeout(0.05_s),
                                            ShooterShoot(&m_cargo, &Xbox)));

  // move intake arm out and spin intake wheels while A is held down,
  // return arm and stop when you let go. (the default mode for Intake)
  frc2::JoystickButton(&Xbox, Button::kA)
      .WhenHeld(IntakeGrabBalls(&m_cargo));
// you can stack commands like this (below).  But in this case, RetractIntake is the default anyway
//      .WhenReleased(RetractIntake(&m_cargo)); 

  // While held, drive robot slower
  frc2::JoystickButton(&Xbox, Button::kBack)
      .WhenHeld(DefaultDrive(
      &m_drive,
      [this] { return Xbox.GetLeftY(); },
      [this] { return Xbox.GetLeftX(); }));

  // this logic will need Camden's explanation to implement
  // limelight aiming. 
  frc2::JoystickButton(&Xbox, Button::kX)
    .WhenHeld(TurnToLimelight(
    &m_drive,
    [this] { return Xbox.GetLeftY()/2; },
    [this] { return Xbox.GetLeftX()/2.5; }));

  frc2::JoystickButton(&Xbox, Button::kStart)
      .WhenHeld(IntakeAllOut(&m_cargo));

  frc2::JoystickButton(&Xbox, Button::kBumperLeft)
      .WhenHeld(ClimberUp(&m_climber, &Xbox));
         
  frc2::JoystickButton(&Xbox, Button::kBumperRight)
      .WhenHeld(ClimberDown(&m_climber, &Xbox));

  frc2::JoystickButton(&Xbox, Button::kStickRight)
      .WhenHeld(DriveResetOdometry(&m_drive, &Xbox));

  // unjam things
  // Run indexer and shooter backwards.  ParallelCommandGroup finishes when all of the commands finish
  // Could change this to ParallelRaceGroup if the indexer commands exit on laserbreaks
  //frc2::JoystickButton(&Xbox, Button::kStart)
      //.WhenHeld(frc2::ParallelCommandGroup{IndexerBackward(&m_indexer),
                                            //ShooterUnjam(&m_cargo)
                                            //});
  // Run indexer and shooter backwards AND burp them out the intake
  //frc2::JoystickButton(&Xbox, Button::kBack)
      //.WhenHeld(frc2::ParallelCommandGroup{IndexerForward(&m_indexer),
                                            //IntakeBottomIn(&m_cargo),
                                            //ShooterUnjam(&m_cargo)
                                            //});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  frc::Trajectory tooballlowpartuno;
   fs::path deployDirectoryuno = frc::filesystem::GetDeployDirectory();
   deployDirectoryuno = deployDirectoryuno / "pathplanner" / "generatedJSON" / "ThreeBallOne.wpilib.json";
   tooballlowpartuno = frc::TrajectoryUtil::FromPathweaverJson(deployDirectoryuno.string());

  frc::Trajectory tooballlowpartdos;
   fs::path deployDirectorydos = frc::filesystem::GetDeployDirectory();
   deployDirectorydos = deployDirectorydos / "pathplanner" / "generatedJSON" / "ThreeBallTwo.wpilib.json";
   tooballlowpartdos = frc::TrajectoryUtil::FromPathweaverJson(deployDirectorydos.string());

  // this sets up the command
  // I also think it fires it off, since this is a CommandHelper?
  frc2::RamseteCommand tooballlowpartunoCommand(
      tooballlowpartuno, 
      [this]() { return m_drive.GetPose(); },
      frc::RamseteController(DriveConstants::kRamseteB,
                             DriveConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive});
    //Reset odometry to starting pose
    m_drive.ResetOdometry(tooballlowpartuno.InitialPose());

  frc2::RamseteCommand tooballlowpartdosCommand(
      tooballlowpartdos, 
      [this]() { return m_drive.GetPose(); },
      frc::RamseteController(DriveConstants::kRamseteB,
                             DriveConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drive.TankDriveVolts(left, right); },
      {&m_drive});

//START COMMENT OUT EXAMPLE S-CURVE
    //no auto 
    return new frc2::SequentialCommandGroup(
      frc2::ParallelRaceGroup( 
        std::move(tooballlowpartunoCommand),     
        IntakeGrabBalls(&m_cargo)),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {} ),
      ShooterAutoShoot(&m_cargo, &Xbox).WithTimeout(3_s),
      frc2::ParallelRaceGroup( 
        std::move(tooballlowpartdosCommand),     
        IntakeGrabBalls(&m_cargo)),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {} ),
      IntakeAllOut(&m_cargo).WithTimeout(0.1_s),
      ShooterAutoShoot(&m_cargo, &Xbox).WithTimeout(4_s));


//STOP COMMENT OUT EXAMPLE S-CURVE     
  // Runs the chosen command in autonomous
  return m_chooser.GetSelected();
}
