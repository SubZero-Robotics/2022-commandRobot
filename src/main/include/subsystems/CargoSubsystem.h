/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "rev/CANSparkMax.h"
#include <ctre/Phoenix.h>
#include <frc/motorcontrol/VictorSP.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/DigitalInput.h>
#include "Constants.h"

class CargoSubsystem : public frc2::SubsystemBase {
 public:
  CargoSubsystem();

  // Subsystem methods go here.

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * One stop solution to handle all of our intaking needs. Now with laser states!
   */
  void GrabBalls();

  /**
   * All index motors in
   */
  void AutomaticIntake();

  /**
   * Bottom indexer wheels forward
   */
  void BottomIn();
  
  /**
   * Bottom, top, indexer wheels backward and intake wheels backwards
   */
  void AllOut();
  
  /**
   * Top indexer wheels forward
   */
  void TopIn();
  
  /**
   * Top indexer wheels backward
   */
  void TopOut();

  /**
   * Move balls towards the target, fast
   */
  void Shoot(); 

  /**
   * Move balls away from the shooter, towards the intake, to clear jams
   */
  void Unjam();  

  /**
   * How fast is the shooter going?
   */
  double GetRPM();

  /**
   * Stop the shooter, indexer, and intake
   */
  void Stop();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  //frc::DoubleSolenoid IntakeArm {frc::PneumaticsModuleType::CTREPCM,2,3};
  //WPI_VictorSPX IntakeWheels{10};
  rev::CANSparkMax IntakeWheels{8, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax BottomIndexer{9, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax TopIndexer{7, rev::CANSparkMax::MotorType::kBrushless};

  frc::DigitalInput TopIntakeLaser {8};  
  frc::DigitalInput BottomIntakeLaser{6};
  bool TopLaserState = 0;
  bool BottomLaserState = 0;

  WPI_TalonSRX Shooter = WPI_TalonSRX(4); 
  WPI_VictorSPX ShooterFollow{5};
  double RPM = 0.0;         // Shooter motor speed
  bool truth = 0;  
};