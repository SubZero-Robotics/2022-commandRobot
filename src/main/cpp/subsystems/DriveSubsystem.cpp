/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/DriveSubsystem.h"
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem() 
{
  // Implementation of subsystem constructor goes here.
  // Stuff you want to happen once, when robot code starts running

  // Initialize each motor with MotionMagic settings
  // Made this a function since we do the same thing four times
  ConfigureMotor(&RightLead);
  ConfigureMotor(&RightFollow);
  ConfigureMotor(&LeftLead);
  ConfigureMotor(&LeftFollow);

  // Invert left side, since DifferentialDrive no longer does it for us
  LeftLead.SetInverted(true);
  LeftFollow.SetInverted(true);

  //Drive train motor grouping start
  RightFollow.Follow(RightLead);
  LeftFollow.Follow(LeftLead);

  // Start encoders at zero
  ResetEncoders();

  // zero gyro
  // Note that this can't happen at power-on when this constructor likely happens
  // because the gyro is calibrating.  Probably want to call this in a later init routine
  ahrs.ZeroYaw();

  // get limelight network tables
  table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  // set camera mode to start with, we don't want the led's on
  table->PutNumber("pipeline", 1);

  // setup for trajectories 
  trajectoryConfig = new frc::TrajectoryConfig(kMaxSpeed,
                               kMaxAcceleration);
  // Create a voltage constraint to ensure we don't accelerate too fast
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics, 10_V);

  // Add kinematics to ensure max speed is actually obeyed
  trajectoryConfig->SetKinematics(DriveConstants::kDriveKinematics);
  // Apply the voltage constraint
  trajectoryConfig->AddConstraint(autoVoltageConstraint);

}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  // Things that happen while robot is running */

  currentrobotAngle = Get2dAngle();
  //Display encoder values in SmartDashboard
  frc::SmartDashboard::PutNumber("2d Angle", (double)currentrobotAngle.Degrees());
  frc::SmartDashboard::PutNumber("Pose X", (double)m_odometry.GetPose().X());
  frc::SmartDashboard::PutNumber("Pose Y", (double)m_odometry.GetPose().Y());
  frc::SmartDashboard::PutNumber("Pose Degrees", (double)m_odometry.GetPose().Rotation().Degrees());
  rEncoder = GetRightEncoder();
  frc::SmartDashboard::PutNumber("Right Encoder", rEncoder);
  lEncoder = GetLeftEncoder();
  frc::SmartDashboard::PutNumber("Left Encoder", lEncoder);
  gyroAngle = ahrs.GetYaw();
  frc::SmartDashboard::PutNumber("gyroAngle", gyroAngle);
  gyroRate = ahrs.GetRate();
  frc::SmartDashboard::PutNumber("gyroRate", gyroRate);
  AverageEncoderDistance = GetAverageEncoderDistance();
  frc::SmartDashboard::PutNumber("Encoder Distance in:", AverageEncoderDistance);

// Get limelight stuff
  tx = table->GetNumber("tx",0.0); 
  frc::SmartDashboard::PutNumber("DifferenceLimelightAngle", tx);
  //float ty = table->GetNumber("ty",0.0); 
  //float ta = table-> GetNumber("ta",0.0);
  //float ts = table-> GetNumber("ts", 0.0);
  tv = table-> GetNumber("tv", 0); //tv is 0 when box is not in view

 m_odometry.Update(currentrobotAngle,
                    units::meter_t(lEncoder*kEncoderDistancePerPulse),
                    units::meter_t(rEncoder*kEncoderDistancePerPulse)); 
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot, true);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  LeftLead.SetVoltage(left);
  RightLead.SetVoltage(right);
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
  RightLead.SetSelectedSensorPosition(0, 0, 10);
  RightFollow.SetSelectedSensorPosition(0, 0, 10);
  LeftLead.SetSelectedSensorPosition(0, 0, 10);
  LeftFollow.SetSelectedSensorPosition(0, 0, 10);
}

// return the average of the two left encoders
double DriveSubsystem::GetLeftEncoder() { 
  return -((LeftLead.GetSelectedSensorPosition()+LeftFollow.GetSelectedSensorPosition())/2.0);
  }

// return the NEGATIVE average of the two right encoders
// Because it's inverted.  Maybe not needed?
double DriveSubsystem::GetRightEncoder() { 
    return -((RightLead.GetSelectedSensorPosition()+RightFollow.GetSelectedSensorPosition())/2.0);
}

// return the average of left and right encoder sets, in feet
double DriveSubsystem::GetAverageEncoderDistance() { 
  return kEncoderDistancePerPulse*((GetLeftEncoder()+GetRightEncoder())/2.0);
}

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t DriveSubsystem::Get2dAngle() {
  return -(units::degree_t)ahrs.GetAngle();
}

units::degree_t DriveSubsystem::GetHeading() {
  // make sure it fits in +/- 180.  Yaw does this, so should be ok.
  return units::degree_t((gyroAngle) * (kGyroReversed ? -1.0 : 1.0));
}

double DriveSubsystem::GetTurnRate() {
  return gyroRate * (kGyroReversed ? -1.0 : 1.0);
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t(-(RightLead.GetSelectedSensorVelocity()*10*kEncoderDistancePerPulse+RightFollow.GetSelectedSensorVelocity()*10*kEncoderDistancePerPulse)/2.0),
          units::meters_per_second_t(-(LeftLead.GetSelectedSensorVelocity()*10*kEncoderDistancePerPulse+LeftFollow.GetSelectedSensorVelocity()*10*kEncoderDistancePerPulse)/2.0)};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, currentrobotAngle);
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

double DriveSubsystem::GetDistance() {
  return Distance;
}

units::degree_t DriveSubsystem::SanitizeAngle(units::degree_t target){
  units::degree_t cleanedAngle = target;
  if ( cleanedAngle >= 180_deg) cleanedAngle -= 360_deg;
  if ( cleanedAngle <= -180_deg) cleanedAngle += 360_deg;
  return cleanedAngle;
}

void DriveSubsystem::ConfigureMotor(WPI_TalonFX *_talon) {
  // Looking at this example:
  // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/C%2B%2B%20Talon%20FX%20(Falcon%20500)/MotionMagic/src/main/cpp/Robot.cpp
  // Sets up MotionMagic parameters inside the motor
  // The exact numbers need to be determined!

  // A reference to all the methods you could call in these motors is:
  // https://store.ctr-electronics.com/content/api/cpp/html/classctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_talon_f_x.html

  // set motor to factory default each time the robot starts, 
  // so that we don't have unexpected things left over
  _talon->ConfigFactoryDefault();

  // Choose the sensor we're using for PID 0 to be the built-in encoders
  // This should be the default anyway, but we'll be sure
  _talon->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,
                                        0, 
                                        10);

  _talon->SetNeutralMode(Brake);

// The commented out commands below are for setting up MotionMagic PIDs
// to run on the motor controller.  Since we're using Ramsete on the RIO
// instead, commenting these out now just in case they cause problems
// although it shouldn't matter
/* Set relevant frame periods to be at least as fast as periodic rate */
//    _talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
//    _talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

    /* Set the peak and nominal outputs */
//    _talon->ConfigNominalOutputForward(0, 10);
//    _talon->ConfigNominalOutputReverse(0, 10);
//    _talon->ConfigPeakOutputForward(1, 10);
//    _talon->ConfigPeakOutputReverse(-1, 10);

    /* Set Motion Magic gains in slot0 - see documentation */
//    _talon->SelectProfileSlot(0, 0);
//    _talon->Config_kF(0, 0.3, 10);
//    _talon->Config_kP(0, 0.1, 10);
//    _talon->Config_kI(0, 0.0, 10);
//    _talon->Config_kD(0, 0.0, 10);

    /* Set acceleration and vcruise velocity - see documentation */
//    _talon->ConfigMotionCruiseVelocity(1500, 10);
//    _talon->ConfigMotionAcceleration(1500, 10);

    /* Zero the sensor */
    _talon->SetSelectedSensorPosition(0, 0, 10);
}

 frc::TrajectoryConfig *DriveSubsystem::GetTrajectoryConfig() {
   return trajectoryConfig;
 }
