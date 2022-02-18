/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSubsystem.h"

#include <frc/smartdashboard/Smartdashboard.h>

#include "Constants.h"

// Constructor, set initial state to in and stopped
IntakeSubsystem::IntakeSubsystem() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();
}

void IntakeSubsystem::Periodic() {
  LaserState = IntakeLaser.Get();
  frc::SmartDashboard::PutBoolean("Intake Laser", LaserState);
}

void IntakeSubsystem::GrabBalls() {
    IntakeWheels.Set(kIntakeSpeed);
    BottomIndexer.Set(kIndexerSpeed);
    if (LaserState) {
        TopIndexer.Set(kIndexerSpeed);
    } else {
        TopIndexer.StopMotor(); }   
    }

void IntakeSubsystem::BottomIn() {
    BottomIndexer.Set(kIndexerSpeed);
}

void IntakeSubsystem::AllOut() {
    TopIndexer.Set(-kIndexerSpeed);
    BottomIndexer.Set(-kIndexerSpeed);
    IntakeWheels.Set(-kIndexerSpeed);
}

void IntakeSubsystem::TopIn() {
    TopIndexer.Set(kIndexerSpeed);
}

void IntakeSubsystem::TopOut() {
    TopIndexer.Set(-kIndexerSpeed);
}

void IntakeSubsystem::AutomaticIntake() {
    BottomIn();
    TopIn();
}

void IntakeSubsystem::Stop() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();
}
