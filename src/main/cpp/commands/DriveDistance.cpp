/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveDistance.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

/* Take a distance in meters, create a pose with the target location that far 
  straight ahead, then schedule a RamsetesCommand to go to that new place */

DriveDistance::DriveDistance(units::meter_t target, 
                             DriveSubsystem* subsystem)
    : m_drive(subsystem), m_distance(target) {
  AddRequirements({subsystem});
}

void DriveDistance::Initialize() {
  // Create a trajectory starting right where we are now and ending m_distance
  // straight ahead
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through this interior waypoint, halfway there
      {frc::Translation2d(m_distance/2.0, 0_m)},
      // End m_distance straight ahead of where we started, facing forward
      frc::Pose2d(m_distance, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      *m_drive->GetTrajectoryConfig());

  // Reset odometry to the starting pose of the trajectory.
  m_drive->ResetOdometry(exampleTrajectory.InitialPose());

  // this sets up the command
  frc2::RamseteCommand DriveDistanceCommand(
      exampleTrajectory, 
      [this]() { return m_drive->GetPose(); },
      frc::RamseteController(DriveConstants::kRamseteB,
                             DriveConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return m_drive->GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { m_drive->TankDriveVolts(left, right); },
      {m_drive});

  // Schedule this new command we just made, followed by a "stop the robot"
    frc2::SequentialCommandGroup(
      std::move(DriveDistanceCommand),
      frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {})
                                  ).Schedule();
}
