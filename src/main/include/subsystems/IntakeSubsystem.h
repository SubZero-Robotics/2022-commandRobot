/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>
#include <frc/motorcontrol/VictorSP.h>

#include <frc2/command/SubsystemBase.h>


class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  // Subsystem methods go here.

  /**
   * Push intake out, starts wheels
   */
  void Extend();

  /**
   * Pulls intake in, stops wheels
   */
  void Retract();

  /**
   * Push intake out, reverse wheels
   */
  void Spit();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  //frc::DoubleSolenoid IntakeArm {frc::PneumaticsModuleType::CTREPCM,2,3};
  WPI_VictorSPX IntakeWheels{10};
};
