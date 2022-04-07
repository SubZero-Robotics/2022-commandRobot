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

#include <frc/Timer.h>
#include <units/time.h>
#include <frc/DriverStation.h>

#include <frc2/command/SubsystemBase.h>

#include <frc/DigitalInput.h>
#include <frc/motorcontrol/Spark.h>
#include "Constants.h"

#include <frc/XboxController.h>
#include "subsystems/DriveSubsystem.h"

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
   * One stop solution to handle all of our intaking needs in auto. Spins up the shooter motors to warm them up.
   */
  void AutoGrabBalls(units::second_t durationOfMove);

  /**
   * @param ledMotorValue the value in percent out to be sent to the Blinkin
   */
  void PutLED(double ledMotorValue);

  /**
   * return bool top laserstate
   */
  bool TopLaserGet();

  /**
   * return bool bottom laserstate
   */
  bool BottomLaserGet();

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
   * Move balls towards the high target, fast
   */
  void Shoot(); 

    /**
   * Move balls towards the high target, fast
   */
  void AutoShoot(); 

  /**
   * Move balls towards the low target, less fast
   */
  void LowShoot(); 

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

  /**
   * Stop the shooter, indexer, and intake
   * @param nextRPM the next RPM vaule to take the rolling average of
   */
  double rollingRPMs(double nextRPM);

 protected:
  frc::Timer m_timer;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  //frc::DoubleSolenoid IntakeArm {frc::PneumaticsModuleType::CTREPCM,2,3};
  //WPI_VictorSPX IntakeWheels{10};
  rev::CANSparkMax IntakeWheels{8, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax BottomIndexer{7, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax TopIndexer{9, rev::CANSparkMax::MotorType::kBrushless};

  frc::DigitalInput TopIntakeLaser {8};  
  frc::DigitalInput BottomIntakeLaser{6};
  frc::Spark led_lights{5};
  bool TopLaserState = 0;
  bool BottomLaserState = 0;
  frc::XboxController Xbox{0};

  WPI_TalonSRX Shooter = WPI_TalonSRX(4); 
  WPI_VictorSPX ShooterFollow{5};
  double RPM = 0.0;         // Shooter motor speed
  bool truth = 0;

  units::second_t m_duration;

  #define numRPMs 5
  double recentRPMs[numRPMs];
  double averageRPMs = 0.0;
};
