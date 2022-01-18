/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/smartdashboard/Smartdashboard.h>

#include "subsystems/ShooterSubsystem.h"

#include "Constants.h" 

// Constructor, set following, direction, and set initial state to in and stopped
ShooterSubsystem::ShooterSubsystem() {

//tie in second motor 
  ShooterFollow.Follow(Shooter);

// PID stuff for shooter
// This runs on the Talon
  Shooter.ConfigFactoryDefault();

  Shooter.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10); 

  Shooter.SetSensorPhase(true);

  Shooter.ConfigNominalOutputForward(0,10);
  Shooter.ConfigNominalOutputReverse(0,10);
  Shooter.ConfigPeakOutputForward(1,10);
  Shooter.ConfigPeakOutputReverse(-1,10);
  
  Shooter.Config_kF(0, kShootF, 10);
  Shooter.Config_kP(0, kShootP, 10);
  Shooter.Config_kI(0, kShootI, 10);
  Shooter.Config_kD(0, kShootD, 10);
}

// Methods

// Implementation of subsystem periodic method goes here.
// for example, publish encoder settings or motor currents to dashboard
void ShooterSubsystem::Periodic() {
  RPM = Shooter.GetSelectedSensorVelocity(0);
  frc::SmartDashboard::PutNumber("RPM", (RPM*600/4096));
}

void ShooterSubsystem::Shoot() {
    Shooter.Set(ControlMode::Velocity, kTargetRPM);
}

void ShooterSubsystem::Unjam() {
    Shooter.Set(ControlMode::PercentOutput, -0.5);
}

void ShooterSubsystem::Stop() {
    Shooter.Set(ControlMode::PercentOutput, 0.0);
}

double ShooterSubsystem::GetRPM() {
    return RPM;
}
