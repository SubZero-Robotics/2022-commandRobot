/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

// Constructor, set initial state to in and stopped
IntakeSubsystem::IntakeSubsystem() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();
}

void IntakeSubsystem::GrabBalls() {
    IntakeWheels.Set(kIntakeSpeed);
}

void IntakeSubsystem::BottomIn() {
    BottomIndexer.Set(kIndexerSpeed);
}

void IntakeSubsystem::BottomOut() {
    BottomIndexer.Set(-kIndexerSpeed/2);
    IntakeWheels.Set(-kIndexerSpeed/2);
}

void IntakeSubsystem::TopIn() {
    TopIndexer.Set(kIndexerSpeed);
}

void IntakeSubsystem::TopOut() {
    TopIndexer.Set(-kIndexerSpeed/2);
}

void IntakeSubsystem::Stop() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();
}
