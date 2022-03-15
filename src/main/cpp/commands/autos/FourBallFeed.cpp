/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/autos/FourBallFeed.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/RamseteCommand.h>

#include "commands/ShooterAutoShoot.h"
#include "commands/IntakeGrabBalls.h"
#include "commands/IntakeAllOut.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>

FourBallFeedRun::FourBallFeedRun(DriveSubsystem* dsubsystem, CargoSubsystem* csubsystem)
    : m_drive{dsubsystem}, m_cargo{csubsystem} {
  AddRequirements({dsubsystem, csubsystem});
}

void FourBallFeedRun::Initialize() {
  // An 2BallsLowPart1 Trajectory
  frc::Trajectory FourBallFeed1;
   fs::path deployDirectoryuno = frc::filesystem::GetDeployDirectory();
   deployDirectoryuno = deployDirectoryuno / "pathplanner" / "generatedJSON" / "ThreeBallOne.wpilib.json";
   FourBallFeed1 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectoryuno.string());

  frc::Trajectory FourBallFeed2;
   fs::path deployDirectorydos = frc::filesystem::GetDeployDirectory();
   deployDirectorydos = deployDirectorydos / "pathplanner" / "generatedJSON" / "FourBallTwo.wpilib.json";
   FourBallFeed2 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectorydos.string());

  frc::Trajectory FourBallFeed3;
   fs::path deployDirectorytres = frc::filesystem::GetDeployDirectory();
   deployDirectorytres = deployDirectorytres / "pathplanner" / "generatedJSON" / "FourBallThree.wpilib.json";
   FourBallFeed2 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectorytres.string());

  // Reset odometry to the starting pose of the trajectory.
  m_drive->ResetOdometry(FourBallFeed1.InitialPose());

  // this sets up the command
  frc2::RamseteCommand FourBallFeed1Command = frc2::RamseteCommand(
      FourBallFeed1, 
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

  // this sets up the command
  frc2::RamseteCommand FourBallFeed2Command = frc2::RamseteCommand(
      FourBallFeed2, 
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

  // this sets up the command
  frc2::RamseteCommand FourBallFeed3Command = frc2::RamseteCommand(
      FourBallFeed3, 
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

  frc2::SequentialCommandGroup* myFourBallFeedAuto = new frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup( 
      std::move(FourBallFeed1Command),     
      IntakeGrabBalls(m_cargo)),
    frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {} ),
    ShooterAutoShoot(m_cargo, &Xbox).WithTimeout(2_s),
    frc2::ParallelRaceGroup( 
      std::move(FourBallFeed2Command),     
      IntakeGrabBalls(m_cargo)),
    frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {} ),
    IntakeGrabBalls(m_cargo).WithTimeout(2_s),
    std::move(FourBallFeed3Command),
    frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {} ),
    IntakeAllOut(m_cargo).WithTimeout(0.1_s),
    ShooterAutoShoot(m_cargo, &Xbox).WithTimeout(4_s));
  myFourBallFeedAuto->Schedule();
}

bool FourBallFeedRun::IsFinished() { return finished; }

void FourBallFeedRun::End(bool interrupted) {

}
