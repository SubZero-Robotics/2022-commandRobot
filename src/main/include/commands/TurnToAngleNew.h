/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/CommandBase.h> 
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>


class TurnToAngleNew
    : public frc2::CommandHelper<frc2::CommandBase, TurnToAngleNew> {
 public:
  /**
   * Creates a new TurnToAngleNew.
   *
   * @param target The number of degrees to turn
   * @param drive  The drive subsystem on which this command will run
   */
  TurnToAngleNew(units::degree_t target);

  void Initialize() override;

 private:
  DriveSubsystem* m_drive;
  units::degree_t m_angle;
  // Leaving this here: once we figure out how to attach a speed scaling to a 
  // RamsetesCommand, we can put it back as a parameter
  double m_speed;  
};
