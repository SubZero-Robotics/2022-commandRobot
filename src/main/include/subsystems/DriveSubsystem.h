/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/AnalogInput.h>
#include <AHRS.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>

#include <frc/drive/DifferentialDrive.h>

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>

#include <frc/trajectory/TrajectoryGenerator.h>

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

/**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  void ArcadeDrive(double fwd, double rot);

/**
   * Controls each side of the drive directly with a voltage.
   *
   * @param left the commanded left output
   * @param right the commanded right output
   */
  void TankDriveVolts(units::volt_t left, units::volt_t right);

/**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

/**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  double GetLeftEncoder();

  /**
   * Gets the right drive encoder.
   * @return the right drive encoder
   */
  double GetRightEncoder();
/**
   * Gets the average distance traveled by both sides, in feet.
   * @return the average of left and right encoders, in feet
   */
  double GetAverageEncoderDistance();

/**
   * Sets the max output of the drive.  Useful for scaling the drive to drive
   * more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  void SetMaxOutput(double maxOutput);

/**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  units::degree_t GetHeading();

/**
   * Translate NavX into Rotation2D values.
   *
   * @return the robot's heading in degrees, coninuous vectorization from 360 to 361
   */
  units::degree_t Get2dAngle();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

/**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

/**
   * Returns the heading to the limelight target
   *
   * @return The heading at which the limelight target sits
   */
  units::degree_t GetLimelightTargetAngle();

  /**
   *
   * Chooses the active limelight pipeline
   */
  void SelectLimelightPipeline(int pipeline);

/**
   *
   * Sets Gyro angle to Zero
   */
  void ZeroGyro(void);

 /**
   * Returns the ultrasonic sensor's distance
   *
   * @return The range from the ultrasonic sensor, in inches
   */
  double GetDistance();


  
 /**
   * make sure target angle is in the right range
   *
   * @return The angle to turn to, put into -180,180 degrees
   */
  units::degree_t SanitizeAngle(units::degree_t target);

 /**
   * set up a motor.  Call this in Init for each motor
   */
  void ConfigureMotor(WPI_TalonFX *_talon);

 /**
   * get the trajectory config for this drive
   */
  frc::TrajectoryConfig *GetTrajectoryConfig();

  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // right motor controllers
  WPI_TalonFX RightLead{12};
  WPI_TalonFX RightFollow{13};
  // left motor controllers
  WPI_TalonFX LeftLead{10};
  WPI_TalonFX LeftFollow{11};

  frc::DifferentialDrive m_drive{RightLead, LeftLead};

  // The default (starting) values for the encoder
  double lEncoder = 0.0;
  double rEncoder = 0.0;

  double AverageEncoderDistance = 0.0;

  // Odometry class for tracking robot pose
  frc::Rotation2d currentrobotAngle; // is zeroed by default
  frc::Pose2d currentRobotPose;      // is also zeroed by default
  // so now use those to initialize odemetry at zero too
  frc::DifferentialDriveOdometry m_odometry{currentrobotAngle,currentRobotPose};

  // Ultrasonic Ranger
  //frc::AnalogInput Ultrasonic{0};
  double Distance = 0.0;    // the ranger distance

  // navx
  double gyroAngle = 0.0;   // What is the angle (degrees) from the gyro?
  double gyroRate = 0.0;    // What is angle change (deg/sec)
  AHRS ahrs{frc::SPI::Port::kMXP};


  // limelight
  float tx = 0.0;           // limelight angle off left/right
  int tv = 1;               // does the limelight have a target?
  // pointer to network tables for limelight stuff
  std::shared_ptr<nt::NetworkTable> table;

  // The drive's config for trajectory
  frc::TrajectoryConfig *trajectoryConfig;
};
