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
    IntakeArm.Set(frc::DoubleSolenoid::Value(1));
    IntakeWheels.Set(ControlMode::PercentOutput, 0.0);
}

void IntakeSubsystem::Extend() {
    IntakeArm.Set(frc::DoubleSolenoid::Value(2));
    IntakeWheels.Set(ControlMode::PercentOutput, kIntakeSpeed);
}

void IntakeSubsystem::Retract() {
    IntakeArm.Set(frc::DoubleSolenoid::Value(1));
    IntakeWheels.Set(ControlMode::PercentOutput, 0.0);
}

void IntakeSubsystem::Spit() {
    IntakeArm.Set(frc::DoubleSolenoid::Value(2));
    IntakeWheels.Set(ControlMode::PercentOutput, -kIntakeSpeed);
}
