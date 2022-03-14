/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/autos/CenterAuto.h"

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

CenterAutoRun::CenterAutoRun(DriveSubsystem* dsubsystem, CargoSubsystem* csubsystem)
    : m_drive{dsubsystem}, m_cargo{csubsystem} {
  AddRequirements({dsubsystem, csubsystem});
}

void CenterAutoRun::Initialize() {
  // An 2BallsLowPart1 Trajectory
  frc::Trajectory tooballlowpartuno;
   fs::path deployDirectoryuno = frc::filesystem::GetDeployDirectory();
   deployDirectoryuno = deployDirectoryuno / "pathplanner" / "generatedJSON" / "ThreeBallOne.wpilib.json";
   tooballlowpartuno = frc::TrajectoryUtil::FromPathweaverJson(deployDirectoryuno.string());

  frc::Trajectory tooballlowpartdos;
   fs::path deployDirectorydos = frc::filesystem::GetDeployDirectory();
   deployDirectorydos = deployDirectorydos / "pathplanner" / "generatedJSON" / "ThreeBallTwo.wpilib.json";
   tooballlowpartdos = frc::TrajectoryUtil::FromPathweaverJson(deployDirectorydos.string());

  // Reset odometry to the starting pose of the trajectory.
  m_drive->ResetOdometry(tooballlowpartuno.InitialPose());

  // this sets up the command
  frc2::RamseteCommand TwoBallUnoCommand = frc2::RamseteCommand(
      tooballlowpartuno, 
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

  frc2::SequentialCommandGroup* myCenterAuto = new frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup( 
      std::move(TwoBallUnoCommand),     
      IntakeGrabBalls(m_cargo)),
    frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {} ),
    ShooterAutoShoot(m_cargo, NULL).WithTimeout(3_s));
  frc2::SequentialCommandGroup* myCenterAuto2 = new frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup( 
      std::move(TwoBallDosCommand),     
      IntakeGrabBalls(m_cargo)),
    frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {} ),
    IntakeAllOut(m_cargo).WithTimeout(0.1_s),
    ShooterAutoShoot(m_cargo, NULL).WithTimeout(4_s));
  myCenterAuto->Schedule();
  myCenterAuto2->Schedule();
}

bool CenterAutoRun::IsFinished() { return finished; }

void CenterAutoRun::End(bool interrupted) {

}
