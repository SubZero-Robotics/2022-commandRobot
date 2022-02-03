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
constexpr double kDeadzone = 0.2;

// How close in inches to rumble while shooting?
constexpr double kRumbleDistance = 85.0;

// The indexer motor speed
constexpr double kIndexerSpeed = 0.5;

// The indexer motor speed
constexpr double kIntakeSpeed = 0.5;

//DriveSubsystem constants

constexpr auto kTrackwidth = 0.69_m; //I have not measured this, it is copied

constexpr int kEncoderCPR = 29860.57; // Counts Per Rotation. TalonFX is 2048
constexpr double kWheelDiameterInches = 6.25;
constexpr double kEncoderDistancePerPulse = //0.000657555;
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterInches * wpi::numbers::pi) /
    static_cast<double>(kEncoderCPR);
// Or: just measure the DistancePerPulse for the whole drivetrain,
// and replace this calculation with a number. 
// Let's do it in feet per tick

// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The Robot Characterization
// Toolsuite provides a convenient tool for obtaining these values for your
// robot.
constexpr auto ks = 0.6506_V;
constexpr auto kv = 0.078629 * 1_V * 1_s / 1_in;
constexpr auto ka = 0.0072005 * 1_V * 1_s * 1_s / 1_in;

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

constexpr auto kMaxTurnRate = 100_deg_per_s;
constexpr auto kMaxTurnAcceleration = 300_deg_per_s / 1_s;

// Shooter constants
constexpr double kTargetRPM = 2500.0; // desired shooter wheel speed
constexpr double kRPM_OK = 100.0;  // if we're within this of the target, it's ok to throw a ball
constexpr double kShootF = 0.1097; //kF for shooter velocity
constexpr double kShootP = 0.22;   //kP for shooter velocity
constexpr double kShootI = 0.0;    //kI for shooter velocity
constexpr double kShootD = 0.0;    //kD for shooter velocity

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
