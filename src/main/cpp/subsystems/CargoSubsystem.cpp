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
}

void CargoSubsystem::Periodic() {
  LaserState = IntakeLaser.Get();
  frc::SmartDashboard::PutBoolean("Intake Laser", LaserState);
}

void CargoSubsystem::GrabBalls() {
    IntakeWheels.Set(kIntakeSpeed);
    BottomIndexer.Set(kIndexerSpeed);
    if (LaserState) {
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

void CargoSubsystem::Stop() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();
}
