/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/motorcontrol/VictorSP.h>
#include <frc/DigitalInput.h>

#include "rev/CANSparkMax.h"

#include <frc2/command/SubsystemBase.h>


class IndexerSubsystem : public frc2::SubsystemBase {
 public:
  IndexerSubsystem();

  // Subsystem methods go here.

/**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Move balls towards the shooter
   */
  void Forward();

  /**
   * Move balls towards the shooter only if the shooter RPM is close to target
   */
  void ForwardCheckRPM();

  /**
   * Move balls away from the shooter, towards the intake
   */
  void Backward();

  /**
   * Stop the shooter
   */
  void Stop();
  
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  //WPI_VictorSPX IndexerT{8};
  //WPI_VictorSPX IndexerB{9};
  /*
  frc::DigitalInput IntakeLaser1 {0};  // eye back by shooter to prevent james
  frc::DigitalInput IntakeLaser2 {1};  // eye near front of indexer to prevent jams going that way
  frc::DigitalInput IntakeLaser3 {2};  // eye at intake, to trigger indexer and slurp ball in
  */
  bool LaserState1 = 0;
  bool LaserState2 = 0;
  bool LaserState3 = 0;
};
