/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "rev/CANSparkMax.h"

#include <frc/XboxController.h>
#include "Constants.h"

#include <ctre/Phoenix.h>
#include <frc/motorcontrol/VictorSP.h>

class ClimberSubsystem : public frc2::SubsystemBase {
 public: 
  ClimberSubsystem();

  // Subsystem methods go here.

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
 
  /**
    * Raise telescoping climber hook
    */
  void Up();

  /**
    * Lower telescoping climber hook
    */
  void Down();
  
  /**
    * Stops the climber
    */
  void Stop();


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax LeftArm{6, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax RightArm{14, rev::CANSparkMax::MotorType::kBrushless};
  WPI_VictorSPX MiddleArmAngle{16};
frc::XboxController Xbox{0};
};