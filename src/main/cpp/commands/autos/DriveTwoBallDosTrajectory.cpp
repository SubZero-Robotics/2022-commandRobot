/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/autos/DriveTwoBallDosTrajectory.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

DriveTwoBallDosTrajectory::DriveTwoBallDosTrajectory(DriveSubsystem* subsystem)
    : m_drive(subsystem) {
  AddRequirements({subsystem});
}

void DriveTwoBallDosTrajectory::Initialize() {
  finished = true;
}

bool DriveTwoBallDosTrajectory::IsFinished() { return finished; }

void DriveTwoBallDosTrajectory::End(bool interrupted) {
  // An 2BallsLowPart2 Trajectory
  frc::Trajectory tooballlowpartdos;
   fs::path deployDirectorydos = frc::filesystem::GetDeployDirectory();
   deployDirectorydos = deployDirectorydos / "pathplanner" / "generatedJSON" / "ThreeBallTwo.wpilib.json";
   tooballlowpartdos = frc::TrajectoryUtil::FromPathweaverJson(deployDirectorydos.string());

  // Reset odometry to the starting pose of the trajectory.
  m_drive->ResetOdometry(tooballlowpartdos.InitialPose());

  // this sets up the command
  frc2::RamseteCommand TwoBallDosCommand = frc2::RamseteCommand(
      tooballlowpartdos, 
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
    frc2::SequentialCommandGroup* myCommandGroup = new frc2::SequentialCommandGroup(
      std::move(TwoBallDosCommand),
      frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {})
                                  );
    myCommandGroup->Schedule();
}