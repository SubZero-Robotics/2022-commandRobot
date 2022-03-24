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
  DisabledInit();

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
  SelectLimelightPipeline(1);

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

void DriveSubsystem::DisabledInit() {
  SetCoast(&RightLead);
  SetCoast(&RightFollow);
  SetCoast(&LeftLead);
  SetCoast(&LeftFollow);
}

void DriveSubsystem::TeleopInit() {
  ConfigureMotor(&RightLead);
  ConfigureMotor(&RightFollow);
  ConfigureMotor(&LeftLead);
  ConfigureMotor(&LeftFollow);
  SelectLimelightPipeline(1);
}

void DriveSubsystem::SetCoast(WPI_TalonFX *_talon) {
   _talon->SetNeutralMode(Coast);

}


void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  // Things that happen while robot is running */

  currentrobotAngle = Get2dAngle();
  //Display encoder values in SmartDashboard
  frc::SmartDashboard::PutNumber("Distance to Target", (double)GetLimelightDistance());
  frc::SmartDashboard::PutNumber("TargetAngle", (double)(gyroAngle+tx));
  frc::SmartDashboard::PutNumber("2d Angle", (double)currentrobotAngle.Degrees());
  frc::SmartDashboard::PutNumber("Pose X", (double)m_odometry.GetPose().X());
  frc::SmartDashboard::PutNumber("Pose Y", (double)m_odometry.GetPose().Y());
  frc::SmartDashboard::PutNumber("Pose Degrees", (double)m_odometry.GetPose().Rotation().Degrees());
  frc::SmartDashboard::PutNumber("Wheel Speeds Meters", (double)((-(RightLead.GetSelectedSensorVelocity()*10*kEncoderDistancePerPulse+RightFollow.GetSelectedSensorVelocity()*10*kEncoderDistancePerPulse)/2.0)
                                  +(-(LeftLead.GetSelectedSensorVelocity()*10*kEncoderDistancePerPulse+LeftFollow.GetSelectedSensorVelocity()*10*kEncoderDistancePerPulse)/2.0))/2);
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
  ty = table->GetNumber("ty",0.0); 
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
  LeftLead.SetVoltage(-left);
  RightLead.SetVoltage(-right);
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
  LOffset = -((LeftLead.GetSelectedSensorPosition()+LeftFollow.GetSelectedSensorPosition())/2.0);
  ROffset = -((RightLead.GetSelectedSensorPosition()+RightFollow.GetSelectedSensorPosition())/2.0);
}

// return the average of the two left encoders
double DriveSubsystem::GetLeftEncoder() { 
  return (-((LeftLead.GetSelectedSensorPosition()+LeftFollow.GetSelectedSensorPosition())/2.0)-LOffset);
  }

// return the NEGATIVE average of the two right encoders
// Because it's inverted.  Maybe not needed?
double DriveSubsystem::GetRightEncoder() { 
    return (-((RightLead.GetSelectedSensorPosition()+RightFollow.GetSelectedSensorPosition())/2.0)-ROffset);
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
    target = units::degree_t((gyroAngle+tx) * (kGyroReversed ? -1.0 : 1.0));
  } else {
    target = units::degree_t((gyroAngle) * (kGyroReversed ? -1.0 : 1.0)); //improvemnet is to remove this have return error so it doesnt turn
  }
  return target; 
}

 units::meter_t DriveSubsystem::GetLimelightDistance() {
  units::meter_t lime_dist = 0_m;
  //calculate distance from the target angle
  lime_dist = 2.1*((kUpHub - kRobotHeight) / tan((ty + kMountAngle) * (3.14159 / 180)) - kIdealDistance);
  return lime_dist;
}

 int DriveSubsystem::GetLimelightTargetValid() {
  return tv; 
}

double DriveSubsystem::GetGyroAngle() {
  return ahrs.GetYaw();
}

void DriveSubsystem::SetTargetAngle(units::degree_t target) {
  TargetAngle = target;
}

void DriveSubsystem::SelectLimelightPipeline(int pipeline){
  // 0 is targeting, 1 is camera
  table->PutNumber("pipeline", pipeline);
}

void DriveSubsystem::ZeroGyro(){
  ahrs.ZeroYaw();
}

units::degree_t DriveSubsystem::LimelightDifferenceAngle() {
  return (units::degree_t)(-tx);
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
  // we're not playing with MotionMagic here though, this comment left in in case we do later

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

    /* Zero the sensor */
    _talon->SetSelectedSensorPosition(0, 0, 10);
}

 frc::TrajectoryConfig *DriveSubsystem::GetTrajectoryConfig() {
   return trajectoryConfig;
 }

 frc::TrajectoryConfig *DriveSubsystem::GetReversedTrajectoryConfig() {
   trajectoryConfig->SetReversed(true);
   return trajectoryConfig;
 }


frc2::SequentialCommandGroup DriveSubsystem::GetRamseteCommand(enum Paths drivePath) {
  // get a pointer to the driveSubsystem, because we need to use it in the command later
  DriveSubsystem *driveSubSystem = this;

// make an empty Trjactory, we will fill it later
frc::Trajectory chosenTrajectory;

// Use the drivePath to pick one of the following trajectories
switch(drivePath) {
  case kScurvePath: // if drivePath == kScurvePath, do the stuff from here to "break"
    // Do an S-curve
    chosenTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      *trajectoryConfig);
    break;  // you've chosen one now, "break" gets you out of the "switch"

  case kStraight1Path:
    // Go straight one meter
    chosenTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these this interior waypoint, just going straight
      {frc::Translation2d(0.5_m, 0_m)},
      // End 1 meter straight ahead of where we started, facing forward
      frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      *trajectoryConfig);
    break;  // always remember the "break:!
  
   default: // default is when the selected drivePath doesn't exist in the list above
  // do nothing: the trajectory doesn't go anywhere, is a safe choice in case you
  // made a new choice but haven't made a "case" for it yet
    chosenTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these this interior waypoint, going nowhere
      {frc::Translation2d(0.0_m, 0_m)},
      // End 0 meters straight ahead of where we started, facing forward
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      *trajectoryConfig);
      // you don't need a "break" at the end of "default" because you're done anyway
}

  // Reset odometry to the starting pose of the trajectory.
  ResetOdometry(chosenTrajectory.InitialPose());

  // create a new RamseteCommand with the chosenTrajectory
  frc2::RamseteCommand chosenCommand(
      chosenTrajectory, 
      [this]() { return GetPose(); },
      frc::RamseteController(DriveConstants::kRamseteB,
                             DriveConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { TankDriveVolts(left, right); },
      {driveSubSystem});

// The RamseteCommand takes several std::functions as arguments, and
// in what looks like a wierd way.
// So, here's what's going on.  We are tying to pass the methods GetPose, 
// GetWheelSpeeds, and TankDriveVolts to RamseteCommand as std::function
// That way, the command can call these DriveSubSystem methods to talk
// to the odometry, speeds, and set the wheel power.
// At the top of this method, we make note of where the real DriveSubSystem is:
//   DriveSubsystem *driveSubSystem = this;
// So, driveSubSystem is a pointer to the robot's actual drive for the 
// final argument as the command subSystem requirements
//
// Look at the first place we try and pass a function using a lambda:
//
//   [this]() { return GetPose(); },
//
// The [] brackets grab something local to use, in this case, it has access to 
// "this", which is the whole local scope.
// The () enclose parameters we're passing to the function.  None
// here, but later you see us making temporary left and right for the volts
// The {} enclose what the function is doing.  In our case, it's 
// calling the GetPose so the RamseteCommand knows the current odometry

  // return the chosen command in a SequentialCommandGroup with a "stop" afterards
  return frc2::SequentialCommandGroup(
    std::move(chosenCommand),
    frc2::InstantCommand([this] { TankDriveVolts(0_V, 0_V); }, {} )
    );
  // you might want to put frc2::InstantCommand([this] { TankDriveVolts(0_V, 0_V); }, {})
  // in after calling this command if the robot doesn't stop
}
