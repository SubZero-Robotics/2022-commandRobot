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

class DriveDistance
    : public frc2::CommandHelper<frc2::CommandBase, DriveDistance> {
 public:
  /**
   * Creates a new DriveDistance.
   *
   * @param target The number of meters to drive
   * @param drive  The drive subsystem on which this command will run
   */
  DriveDistance(units::meter_t target, DriveSubsystem* subsystem);

  void Initialize() override;

 private:
  DriveSubsystem* m_drive;
  units::meter_t m_distance;
  // Leaving this here: once we figure out how to attach a speed scaling to a 
  // RamsetesCommand, we can put it back as a parameter
  double m_speed;  
};
