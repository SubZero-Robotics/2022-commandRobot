/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/CargoSubsystem.h"

#include <frc/smartdashboard/Smartdashboard.h>

#include "Constants.h"

// Constructor, set initial state to in and stopped
CargoSubsystem::CargoSubsystem() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();

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

void CargoSubsystem::Periodic() {
  TopLaserState = TopIntakeLaser.Get();
  frc::SmartDashboard::PutBoolean("Top Intake Laser", TopLaserState);

  BottomLaserState = BottomIntakeLaser.Get();
  frc::SmartDashboard::PutBoolean("Bottom Intake Laser", BottomLaserState);

  
  RPM = -Shooter.GetSelectedSensorVelocity(0);
  frc::SmartDashboard::PutNumber("RPM", (RPM*600/4096));
  frc::SmartDashboard::PutBoolean("INTAKE WOULD BE SPINNING", truth);
}

void CargoSubsystem::GrabBalls() {
    IntakeWheels.Set(kIntakeSpeed);
    BottomIndexer.Set(kIndexerSpeed);
    if (TopLaserState) {
        TopIndexer.Set(kIndexerSpeed);
    } else {
        TopIndexer.StopMotor(); }   
    }

void CargoSubsystem::BottomIn() {
    BottomIndexer.Set(kIndexerSpeed);
}

void CargoSubsystem::AllOut() {
    TopIndexer.Set(-kIndexerSpeed);
    BottomIndexer.Set(-kIndexerSpeed);
    IntakeWheels.Set(-kIndexerSpeed);
}

void CargoSubsystem::TopIn() {
    TopIndexer.Set(kIndexerSpeed);
}

void CargoSubsystem::TopOut() {
    TopIndexer.Set(-kIndexerSpeed);
}

void CargoSubsystem::AutomaticIntake() {
    BottomIn();
    TopIn();
}

void CargoSubsystem::Shoot() {
    Shooter.Set(ControlMode::Velocity, kTargetRPM);
    if ((-Shooter.GetSelectedSensorVelocity(0) + kRPM_OK) >= kTargetRPM) {
      truth = true;
      BottomIndexer.Set(kIndexerSpeed);
      TopIndexer.Set(kIndexerSpeed);
    } else {
      truth = 0;
      BottomIndexer.StopMotor();
      TopIndexer.StopMotor();
    }

}

void CargoSubsystem::Unjam() {
    Shooter.Set(ControlMode::PercentOutput, -0.5);
}

void CargoSubsystem::Stop() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();
    Shooter.Set(ControlMode::PercentOutput, 0.0);    
}

double CargoSubsystem::GetRPM() {
    return RPM;
}