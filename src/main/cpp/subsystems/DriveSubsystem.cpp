/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/DriveSubsystem.h"
//#include <frc/Encoder.h>

DriveSubsystem::DriveSubsystem() {
  // Implementation of subsystem constructor goes here.

  // Invert right side, since DifferentialDrive no longer does it for us
  RightLead.SetInverted(true);
  RightFollow.SetInverted(true);

  //Drive train motor grouping start
  RightFollow.Follow(RightLead);
  LeftFollow.Follow(LeftLead);

  // Set the distance per pulse for the encoders
  //m_leftEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);
  //m_rightEncoder.SetDistancePerPulse(kEncoderDistancePerPulse); 

  //ResetEncoders();

  // zero gyro
  // Note that this can't happen at power-on when this constructor likely happens
  // because the gyro is calibrating.  Probably want to call this in a later init routine
  ahrs.ZeroYaw();

  // get limelight network tables
  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  // set camera mode to start with, we don't want the led's on
  table->PutNumber("pipeline", 1);
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  // for example, publish encoder settings or motor currents to dashboard
  /*Distance = Ultrasonic.GetVoltage()*1000.0*(1.0/0.977)*(1.0/25.4);
  frc::SmartDashboard::PutNumber("Distance", Distance);*/

  //Display encoder values in SmartDashboard
  rEncoder = -RightLead.GetSelectedSensorPosition();
  frc::SmartDashboard::PutNumber("Right Encoder", rEncoder);
  lEncoder = LeftLead.GetSelectedSensorPosition();
  frc::SmartDashboard::PutNumber("Left Encoder", lEncoder);

  gyroAngle = ahrs.GetYaw();
  frc::SmartDashboard::PutNumber("gyroAngle", gyroAngle);
  gyroRate = ahrs.GetRate();
  frc::SmartDashboard::PutNumber("gyroRate", gyroRate);

// Get limelight stuff
  tx = table->GetNumber("tx",0.0); 
  frc::SmartDashboard::PutNumber("DifferenceLimelightAngle", tx);
  //float ty = table->GetNumber("ty",0.0); 
  //float ta = table-> GetNumber("ta",0.0);
  //float ts = table-> GetNumber("ts", 0.0);
  tv = table-> GetNumber("tv", 0); //tv is 0 when box is not in view 
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot, true);
}
/*
void DriveSubsystem::ResetEncoders() {
 m_leftEncoder.Reset();
 m_rightEncoder.Reset();
}

double DriveSubsystem::GetAverageEncoderDistance() {
  return (m_leftEncoder.GetDistance() + m_rightEncoder.GetDistance()) / 2.0;
}

frc::Encoder& DriveSubsystem::GetLeftEncoder() { return m_leftEncoder; }

frc::Encoder& DriveSubsystem::GetRightEncoder() { return m_rightEncoder; }

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}*/

units::degree_t DriveSubsystem::GetHeading() {
  // make sure it fits in +/- 180.  Yaw does this, so should be ok.
  return units::degree_t((gyroAngle) * (kGyroReversed ? -1.0 : 1.0));
}

double DriveSubsystem::GetTurnRate() {
  return gyroRate * (kGyroReversed ? -1.0 : 1.0);
}

 units::degree_t DriveSubsystem::GetLimelightTargetAngle() {
  units::degree_t target = 0_deg;
  // give a number if we have a target, else just return where we're already heading
  // could change this to turn and seek a target
  if (tv) {
    units::degree_t((gyroAngle+tx) * (kGyroReversed ? -1.0 : 1.0));
  } else {
    units::degree_t((gyroAngle) * (kGyroReversed ? -1.0 : 1.0)); //improvemnet is to remove this have return error so it doesnt turn
  }
  return target; 
}

void DriveSubsystem::SelectLimelightPipeline(int pipeline){
  // 0 is targeting, 1 is camera
  table->PutNumber("pipeline", pipeline);
}

void DriveSubsystem::ZeroGyro(){
  ahrs.ZeroYaw();
}

/*double DriveSubsystem::GetDistance() {
  return Distance;
}*/

units::degree_t DriveSubsystem::SanitizeAngle(units::degree_t target){
  units::degree_t cleanedAngle = target;
  if ( cleanedAngle >= 180_deg) cleanedAngle -= 360_deg;
  if ( cleanedAngle <= -180_deg) cleanedAngle += 360_deg;
  return cleanedAngle;
}