/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/autos/TwoBallUp.h"

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

TwoBallUpRun::TwoBallUpRun(DriveSubsystem* dsubsystem, CargoSubsystem* csubsystem)
    : m_drive{dsubsystem}, m_cargo{csubsystem} {
  AddRequirements({dsubsystem, csubsystem});
}

void TwoBallUpRun::Initialize() {
  // An Top Auto Two Ball Trajectory
  frc::Trajectory TwoBallUp1;
   fs::path deployDirectoryuno = frc::filesystem::GetDeployDirectory();
   deployDirectoryuno = deployDirectoryuno / "pathplanner" / "generatedJSON" / "Top Auto.wpilib.json";
   TwoBallUp1 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectoryuno.string());
   
  // Reset odometry to the starting pose of the trajectory.
  m_drive->ResetOdometry(TwoBallUp1.InitialPose());

  // this sets up the command
  frc2::RamseteCommand TwoBallUp1Command = frc2::RamseteCommand(
      TwoBallUp1, 
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

  frc2::SequentialCommandGroup* myTwoBallUpAuto = new frc2::SequentialCommandGroup(
    frc2::ParallelRaceGroup( 
      std::move(TwoBallUp1Command),     
      IntakeGrabBalls(m_cargo)),
    frc2::InstantCommand([this] { m_drive->TankDriveVolts(0_V, 0_V); }, {} ),
    IntakeAllOut(m_cargo).WithTimeout(0.1_s), 
    ShooterAutoShoot(m_cargo, &Xbox).WithTimeout(3_s));
  myTwoBallUpAuto->Schedule();
}

bool TwoBallUpRun::IsFinished() { return finished; }

void TwoBallUpRun::End(bool interrupted) {

}
