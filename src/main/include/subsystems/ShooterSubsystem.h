/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/motorcontrol/VictorSP.h>

#include "rev/CANSparkMax.h"

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  // Subsystem methods go here.

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

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
  WPI_TalonSRX Shooter = WPI_TalonSRX(4); 
  WPI_VictorSPX ShooterFollow{5};
  //rev::CANSparkMax Shooter{5, rev::CANSparkMax::MotorType::kBrushless};
  //rev::CANSparkMax ShooterFollow{6, rev::CANSparkMax::MotorType::kBrushless};
  double RPM = 0.0;         // Shooter motor speed
};
