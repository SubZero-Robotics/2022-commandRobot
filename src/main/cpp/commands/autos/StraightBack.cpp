/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/autos/StraightBack.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/RamseteCommand.h>

#include "commands/ShooterAutoShootOne.h"
#include "commands/ShooterShoot.h"
#include "commands/IntakeGrabBalls.h"
#include "commands/IntakeAllOut.h"
#include "commands/LimelightTimedCopy.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>

StraightBackRun::StraightBackRun(DriveSubsystem* dsubsystem, CargoSubsystem* csubsystem)
    : m_drive{dsubsystem}, m_cargo{csubsystem} {
  AddRequirements({dsubsystem, csubsystem});
}

void StraightBackRun::Initialize() {
  // An Top Auto Two Ball Trajectory
  frc::Trajectory StraightBack1;
   fs::path deployDirectoryuno = frc::filesystem::GetDeployDirectory();
   deployDirectoryuno = deployDirectoryuno / "pathplanner" / "generatedJSON" / "StraightBack.wpilib.json";
   StraightBack1 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectoryuno.string());
   
  // Reset odometry to the starting pose of the trajectory.
  m_drive->ResetOdometry(StraightBack1.InitialPose());

  // this sets up the command
  frc2::RamseteCommand StraightBack1Command = frc2::RamseteCommand(
      StraightBack1, 
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

  frc2::SequentialCommandGroup* myStraightBackAuto = new frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup( 
      std::move(StraightBack1Command),     
      IntakeGrabBalls(m_cargo)),
    frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {} ), 
    LimelightTimedCopy(m_drive,
    [this] { return Xbox.GetLeftY(); },
    [this] { return Xbox.GetLeftX(); }).WithTimeout(3.5_s),
    ShooterAutoShootOne(m_cargo, &Xbox),
    frc2::WaitCommand(0.4_s),
    ShooterShoot(m_cargo, &Xbox).WithTimeout(5_s));
  myStraightBackAuto->Schedule();
}

bool StraightBackRun::IsFinished() { return finished; }

void StraightBackRun::End(bool interrupted) {

}
