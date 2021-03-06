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
#include "rev/ColorSensorV3.h"
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
  void AutoGrabBalls();

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
   * Intake arm PID goes to down position with gap
   */
  void IntakeDown();
  
  /**
   * Bottom, top, indexer wheels backward and intake wheels backwards
   */
  void AllOut();

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
   * How fast is the shooter going?
   */
  double GetRPM();

  /**
   * Stop the shooter, indexer, and intake, return intake arm PID to upright
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
  WPI_TalonSRX IntakeArm = WPI_TalonSRX(15); 

  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  bool ballCorrectColor = true;

  frc::DigitalInput TopIntakeLaser {8};  
  frc::DigitalInput BottomIntakeLaser{6};
  frc::Spark led_lights{1};
  bool TopLaserState = 0;
  bool BottomLaserState = 0;
  frc::XboxController Xbox{0};

  WPI_TalonSRX Shooter = WPI_TalonSRX(4); 
  WPI_VictorSPX ShooterFollow{5};
  double RPM = 0.0;         // Shooter motor speed
  bool truth = 0;

  bool firstPass = true;
};
