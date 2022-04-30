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
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/RamseteCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  void DisabledInit();

  void TeleopInit();

  void SetCoast(WPI_TalonFX *_talon);

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
   * @return The current pose of the robot
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
   * Returns the heading to the limelight target
   *
   * @return The heading at which saved limelight target is
   */
  double GetGyroAngle();

/**
   * Sets the heading to the limelight target
   *
   * @param target The heading at which saved limelight target is
   */
  void SetTargetAngle(units::degree_t target);


/**
   * Returns if the limelight has a target
   *
   * @return 1 for target, 0 for not
   */
  int GetLimelightTargetValid();

  /**
   *
   * Chooses the active limelight pipeline
   * 
   * @param pipeline which limlelight pipeline to turn on
   */
  void SelectLimelightPipeline(int pipeline);

/**
   *
   * Sets Gyro angle to Zero
   */
  void ZeroGyro(void);

 /**
   * Returns tx value in a degree wrapper
   *
   * @return Return only the tx value, can be in ~ range -20_deg to 20_deg
   */
  units::degree_t LimelightDifferenceAngle();
  
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
   * This example uses code from the aiming and range adjustment examples and puts everything together into one simple function. Using this, you can get your robot “close” and then use code to automatically aim and drive to the correct distance.
   */
  void LimelightTimedCopy(double fwd, double rot);

 /**
   * get the trajectory config for this drive
   */
  frc::TrajectoryConfig *GetTrajectoryConfig();

 /**
   * get a reversed trajectory config for this drive
   */
  frc::TrajectoryConfig *GetReversedTrajectoryConfig();

  /**
   * Returns the distance to the limelight target
   *
   * @return The distance at from where we want to be in relation to the target
   */
  double GetLimelightDistance();

 /**
   * Choose a path
   * 
   * @param drivePath the path you're picking
   * @return a RamseteCommand to drive that path
  */
  frc2::SequentialCommandGroup GetRamseteCommand(enum Paths drivePath);
  
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
  double ROffset = 0.0;
  double LOffset = 0.0;

  double AverageEncoderDistance = 0.0;

  frc::SlewRateLimiter<units::scalar> decelfilter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> accelfilter{3 / 1_s};
  double previousPercentage = 0.0;

  // Odometry class for tracking robot pose
  frc::Rotation2d currentrobotAngle; // is zeroed by default
  frc::Pose2d currentRobotPose;      // is also zeroed by default
  // so now use those to initialize odemetry at zero too
  frc::DifferentialDriveOdometry m_odometry{currentrobotAngle,currentRobotPose};

  // navx
  double gyroAngle = 0.0;   // What is the angle (degrees) from the gyro?
  double gyroRate = 0.0;    // What is angle change (deg/sec)
  AHRS ahrs{frc::SPI::Port::kMXP};

  // limelight
  float tx = 0.0;           // limelight angle off left/right
  int tv = 1;               // does the limelight have a target?
  float ty = 0.0;           // limelight distance calculation
  units::degree_t TargetAngle = 0.0_deg;  // Saved value of an angle to turn to

  //TurnToAnglePID
  frc2::PIDController TurnToAngle{0.0396, 0.132, 0.00297};

  // pointer to network tables for limelight stuff
  std::shared_ptr<nt::NetworkTable> table;

  // The drive's config for trajectory
  frc::TrajectoryConfig *trajectoryConfig;
};
