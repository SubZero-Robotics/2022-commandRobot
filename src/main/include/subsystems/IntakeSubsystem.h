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

#include "rev/CANSparkMax.h"

#include <frc2/command/SubsystemBase.h>


class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  // Subsystem methods go here.

  /**
   * Push intake out, starts wheels
   */
  void Spin();

  /**
   * Pulls intake in, stops wheels
   */
  void Retract();

  /**
   * Push intake out, reverse wheels
   */
  void Spit();

  
  /**
   * Move balls towards the target, fast
   */
  void Shoot(); 

  /**
   * Move balls away from the shooter, towards the intake, to clear jams
   */
  void Unjam();

  /**
   * Stop the shooter
   */
  void Stop();

  /**
   * How fast is it going?
   */
  double GetRPM();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  //frc::DoubleSolenoid IntakeArm {frc::PneumaticsModuleType::CTREPCM,2,3};
  //WPI_VictorSPX IntakeWheels{10};
  rev::CANSparkMax IntakeWheels{8, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax BottomIndexer{9, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax TopIndexer{7, rev::CANSparkMax::MotorType::kBrushless};

  WPI_VictorSPX Shooter = WPI_VictorSPX(4); 
  WPI_VictorSPX ShooterFollow{5};
};
