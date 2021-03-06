/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

// #include <units/units.h> 
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// The deadzone for the joystick
namespace DriveConstants {
  constexpr auto kTrackWidth = 0.6096_m;
  constexpr int kEncoderCPR = 29860.57; // Counts Per Rotation. TalonFX is 2048
  constexpr double kWheelDiameterMeters = 0.15875;
  constexpr double kEncoderDistancePerPulse = 0.00001699323;
    // Assumes the encoders are directly mounted on the wheel shafts
    //((kWheelDiameterMeters * wpi::numbers::pi) /
    //static_cast<double>(kEncoderCPR)); ////////now in feet by dividing by 12
// Or: just measure the DistancePerPulse for the whole drivetrain,
// and replace this calculation with a number. 

// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The Robot Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
  constexpr auto ks = 0.6416_V;
  constexpr auto kv = 3.1057 * 1_V * 1_s / 1_m;
  constexpr auto ka = 0.29699 * 1_V * 1_s * 1_s / 1_m;
  extern const frc::DifferentialDriveKinematics kDriveKinematics;

  constexpr double kPDriveVel = 1.6686;

  constexpr double kRamseteB = 2.0;
  constexpr double kRamseteZeta = 0.7;
}
constexpr double kDeadzone = 0.1;

// How close in inches to rumble while shooting?
constexpr double kRumbleDistance = 85.0;

// The indexer motor speed
constexpr double kIndexerSpeed = 0.45;

// The indexer motor speed
constexpr double kIntakeSpeed = 0.95;

//DriveSubsystem constants

// Turning constants
constexpr bool kGyroReversed = false;

constexpr double kStabilizationP = 0.12; //tune This to start oscillating.  0.2 made 0.667s periods .12
constexpr double kStabilizationI = 0.36; //Then tune this to stop the osolating .36
constexpr double kStabilizationD = 0.01; //Finaly tune this to fix final error  .01

constexpr double kTurnP = 0.10; //tune This to start oscillating.  0.2 made 0.667s periods .12
constexpr double kTurnI = 0.30; //Then tune this to stop the osolating .36
constexpr double kTurnD = 0.01; //Finaly tune this to fix final error  .01

constexpr auto kTurnTolerance = 5_deg;
constexpr auto kTurnRateTolerance = 10_deg_per_s;

constexpr auto kMaxSpeed = 1.5_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;

constexpr auto kMaxTurnRate = 70_deg_per_s;
constexpr auto kMaxTurnAcceleration = 200_deg_per_s / 1_s;


// Shooter constants
constexpr double kHighTargetRPM = 1150.0; // desired shooter wheel speed
constexpr double kLowTargetRPM = 690.0; // desired shooter wheel speed
constexpr double kRPM_OK = 0.0;  // if we're within this of the target, it's ok to throw a ball
constexpr double kShootF = 0.01; //kF for shooter velocity
constexpr double kShootP = 0.09;   //kP for shooter velocity
constexpr double kShootI = 0.0;    //kI for shooter velocity
constexpr double kShootD = 3.0;    //kD for shooter velocity

// Robot/Field Constants
constexpr double kRobotHeight = 0.94;// Height of limelight
constexpr double kUpHub = 2.64;      // height of upper target
constexpr double kMountAngle = 25.0; // Limelight angle off horizintal
constexpr double kIdealDistance = 2.976;

// XboxController enums.  Since the Trigger stuff works on the base Joystick class, not the
// Xbox extension, these are undefined where we want to use them.  So, flat-out copied them 
// here for reference
enum  	Button {
  kBumperLeft = 5, kBumperRight = 6, kStickLeft = 9, kStickRight = 10,
  kA = 1, kB = 2, kX = 3, kY = 4,
  kBack = 7, kStart = 8
};
 
enum  	Axis {
  kLeftX = 0, kRightX = 4, kLeftY = 1, kRightY = 5,
  kLeftTrigger = 2, kRightTrigger = 3
};

// A list of paths to pick from
// Note that you don't have to manually set them equal to a number like "Axis" above,
// that happens automaticall if you're ok with starting at 0 and counting up
// So, perfectly ok to just throw kJacksMostExcellentPath on the end of this list
// and you'll be able to use it later
//
// Why start the names with "k"?  that doesn't actually do anything, just reminds us that
// the thing is a constant so you should probably look for it in this file
enum    Paths {
  kScurvePath, kStraight1Path, kTwoBallsUnoPath, kTwoBallsDosPath
};
