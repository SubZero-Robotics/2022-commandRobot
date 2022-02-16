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
    //IntakeArm.Set(frc::DoubleSolenoid::Value(1));
    IntakeWheels.StopMotor();
}

void IntakeSubsystem::GrabBalls() {
    //IntakeArm.Set(frc::DoubleSolenoid::Value(2));
    IntakeWheels.Set(kIntakeSpeed);
    BottomIndexer.Set(kIntakeSpeed);
    TopIndexer.Set(kIntakeSpeed);
}

void IntakeSubsystem::Spit() {
    //IntakeArm.Set(frc::DoubleSolenoid::Value(2));
    //IntakeWheels.Set(ControlMode::PercentOutput, -kIntakeSpeed);
}


void IntakeSubsystem::IndexForward() {
    //IndexerB.Set(ControlMode::PercentOutput, -kIndexerSpeed);
}

void IntakeSubsystem::IndexForwardCheckRPM() {
// get RPMs from network tables to avoid a ShooterSubsystem dependancy
// Here we check that RPMS are within kRPM_OK on the low side.  Anything faster is ok
// If you're worried about goign to fast, change this calculation
    //if ((frc::SmartDashboard::GetNumber("RPM",0.0) + kRPM_OK) >= kTargetRPM) 
        //IndexerB.Set(ControlMode::PercentOutput, -kIndexerSpeed);
}

void IntakeSubsystem::IndexBackward() {
    //IndexerB.Set(ControlMode::PercentOutput, kIndexerSpeed);
}

void IntakeSubsystem::Stop() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();
}
