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
#include "commands/ShooterStop.h"
#include "commands/ShooterUnjam.h"

#include "commands/ClimberUp.h"
#include "commands/ClimberDown.h"
#include "commands/ClimberHighClimb.h"
#include "commands/ClimberStop.h"

#include "commands/DefaultDrive.h"
#include "commands/TurnToLimelight.h"
#include "commands/TurnToAngle.h"
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
      [this] { return Xbox.GetLeftY(); },
      [this] { return Xbox.GetLeftX(); }));

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
      .WhenHeld(ShooterLowShoot(&m_cargo, &Xbox));

  //Spin up shooter motor for high while pressed, and rumble controller if you're too close
  frc2::JoystickButton(&Xbox, Button::kY)
      .WhenHeld(ClimberHighClimb(&m_climber, &Xbox)); 

  // move intake arm out and spin intake wheels while A is held down,
  // return arm and stop when you let go. (the default mode for Intake)
  frc2::JoystickButton(&Xbox, Button::kA)
      .WhenHeld(IntakeGrabBalls(&m_cargo));
// you can stack commands like this (below).  But in this case, RetractIntake is the default anyway
//      .WhenReleased(RetractIntake(&m_cargo)); 

  // While held, drive robot slower
  frc2::JoystickButton(&Xbox, Button::kX)
      .WhenHeld(DefaultDrive(
      &m_drive,
      [this] { return Xbox.GetLeftY()/2; },
      [this] { return Xbox.GetLeftX()/2; }));

  frc2::JoystickButton(&Xbox, Button::kStart)
      .WhenHeld(IntakeAllOut(&m_cargo));

  // Climber deployment and winch: Y button plus right stick
  //frc2::JoystickButton(&Xbox, Axis::kRightY) 
      //.WhenHeld(ClimberUp(&m_climber, &Xbox));

  frc2::JoystickButton(&Xbox, Button::kBumperLeft)
      .WhenHeld(ClimberUp(&m_climber, &Xbox));

  frc2::JoystickButton(&Xbox, Button::kStickRight)
      .WhenHeld(DriveResetOdometry(&m_drive, &Xbox));
  
  // this logic will need Camden's explanation to implement
  // limelight aiming.  
  frc2::JoystickButton(&Xbox, Button::kBumperRight)
      .WhenHeld(ClimberDown(&m_climber, &Xbox));

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
  frc::Trajectory trajectory;
   fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
   deployDirectory = deployDirectory / "pathplanner" / "generatedJSON" / "Cheeks.wpilib.json";
   trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, 1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 1_m, frc::Rotation2d(0_deg)),
      // Pass the config
      *m_drive.GetTrajectoryConfig());

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(trajectory.InitialPose());

  // this sets up the command
  // I also think it fires it off, since this is a CommandHelper?
  frc2::RamseteCommand ScurveCommand(
      trajectory, 
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

    m_drive.ResetOdometry(trajectory.InitialPose());
//START COMMENT OUT EXAMPLE S-CURVE
    //no auto 
    /*return new frc2::SequentialCommandGroup(
      std::move(ScurveCommand),
      frc2::InstantCommand([this] { m_drive.TankDriveVolts(0_V, 0_V); }, {} ));*/
//STOP COMMENT OUT EXAMPLE S-CURVE     
  // Runs the chosen command in autonomous
  return m_chooser.GetSelected();
}
