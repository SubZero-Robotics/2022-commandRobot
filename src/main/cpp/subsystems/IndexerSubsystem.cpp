/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <cmath>

#include <frc/smartdashboard/Smartdashboard.h>

#include "subsystems/IndexerSubsystem.h"

#include "Constants.h"

// Constructor, set following, direction, and set initial state to in and stopped
IndexerSubsystem::IndexerSubsystem() {
    IndexerT.Follow(IndexerB);
    IndexerT.SetInverted(true); 
    IndexerB.Set(ControlMode::PercentOutput, 0.0);
}

// Methods

// Implementation of subsystem periodic method goes here.
// for example, publish encoder settings or motor currents to dashboard
void IndexerSubsystem::Periodic() {
// get the electric eye statuses
  LaserState1 = IntakeLaser1.Get();
  frc::SmartDashboard::PutBoolean("Laser1", LaserState1);
  LaserState2 = IntakeLaser2.Get();
  frc::SmartDashboard::PutBoolean("Laser2", LaserState2);
  LaserState3 = IntakeLaser3.Get();
  frc::SmartDashboard::PutBoolean("Laser3", LaserState3);
}

void IndexerSubsystem::Forward() {
    IndexerB.Set(ControlMode::PercentOutput, -kIndexerSpeed);
}

void IndexerSubsystem::ForwardCheckRPM() {
// get RPMs from network tables to avoid a ShooterSubsystem dependancy
// Here we check that RPMS are within kRPM_OK on the low side.  Anything faster is ok
// If you're worried about goign to fast, change this calculation
    if ((frc::SmartDashboard::GetNumber("RPM",0.0) + kRPM_OK) >= kTargetRPM) 
        IndexerB.Set(ControlMode::PercentOutput, -kIndexerSpeed);
}

void IndexerSubsystem::Backward() {
    IndexerB.Set(ControlMode::PercentOutput, kIndexerSpeed);
}

void IndexerSubsystem::Stop() {
    IndexerB.Set(ControlMode::PercentOutput, 0.0);
}
