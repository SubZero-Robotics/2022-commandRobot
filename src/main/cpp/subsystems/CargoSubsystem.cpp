/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/CargoSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

// Constructor, set initial state to in and stopped
CargoSubsystem::CargoSubsystem() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();
//reset follow motor
  ShooterFollow.ConfigFactoryDefault();
//tie in second motor 
  ShooterFollow.Follow(Shooter);

// PID stuff for shooter
// This runs on the Talon
  Shooter.ConfigFactoryDefault();
  
  Shooter.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10); 

  Shooter.SetSensorPhase(true);
  
  Shooter.ConfigNominalOutputForward(0,10);
  Shooter.ConfigNominalOutputReverse(0,10);
  Shooter.ConfigPeakOutputForward(1,10);
  Shooter.ConfigPeakOutputReverse(-1,10);
  
  Shooter.Config_kF(0, kShootF, 10);
  Shooter.Config_kP(0, kShootP, 10);
  Shooter.Config_kI(0, kShootI, 10);
  Shooter.Config_kD(0, kShootD, 10);    

//Intake arm motor setting
  IntakeArm.ConfigFactoryDefault();
  
  IntakeArm.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10); 
  IntakeArm.ConfigReverseLimitSwitchSource(
					LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
					LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
					10);


  IntakeArm.SetSensorPhase(true);

  /* Set relevant frame periods to be at least as fast as periodic rate */
  IntakeArm.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  IntakeArm.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

  /* Set the peak and nominal outputs */
  IntakeArm.ConfigNominalOutputForward(0,10);
  IntakeArm.ConfigNominalOutputReverse(0,10);
  IntakeArm.ConfigPeakOutputForward(1,10);
  IntakeArm.ConfigPeakOutputReverse(-1,10);

  /* Set Motion Magic gains in slot0 - see documentation */
  IntakeArm.SelectProfileSlot(0, 0);
  IntakeArm.Config_kF(0, 0.9, 10);
  IntakeArm.Config_kP(0, 0.35, 10);
  IntakeArm.Config_kI(0, 0.0, 10);
  IntakeArm.Config_kD(0, 6.0, 10);

  /* Set acceleration and vcruise velocity - see documentation */
  IntakeArm.ConfigMotionCruiseVelocity(1500, 10);
  IntakeArm.ConfigMotionAcceleration(2400, 10);

  /* Zero the sensor */
  IntakeArm.SetSelectedSensorPosition(0, 0, 10);
}

void CargoSubsystem::Periodic() {
  TopLaserState = TopIntakeLaser.Get();
  frc::SmartDashboard::PutBoolean("Top Intake Laser", TopLaserState);
  BottomLaserState = BottomIntakeLaser.Get();
  frc::SmartDashboard::PutBoolean("Bottom Intake Laser", BottomLaserState);
  frc::SmartDashboard::PutBoolean("BallCorrectColor", ballCorrectColor);

  static frc::DriverStation::Alliance AllianceColor = frc::DriverStation::GetAlliance();
  frc::Color detectedColor = m_colorSensor.GetColor();
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  if (frc::DriverStation::IsDisabled() != true && frc::DriverStation::IsAutonomous() == false) {
    switch (AllianceColor) {
    case frc::DriverStation::kRed:
      if ((detectedColor.blue / detectedColor.red) > 1.1) {
        ballCorrectColor = false;
        frc::SmartDashboard::PutBoolean("is red", false);
      } else {
        ballCorrectColor = true;
        frc::SmartDashboard::PutBoolean("is red", true);
      }
      break;
    case frc::DriverStation::kBlue:
      if ((detectedColor.red / detectedColor.blue) > 1.2) {
        ballCorrectColor = false;
        frc::SmartDashboard::PutBoolean("is blue", false);
      } else {
        ballCorrectColor = true;
        frc::SmartDashboard::PutBoolean("is blue", true);
      }
      break;
    case frc::DriverStation::kInvalid:
    default:
        ballCorrectColor = true;
      break;
    }
  }

  RPM = abs(Shooter.GetSelectedSensorVelocity(0));
  frc::SmartDashboard::PutNumber("RPM", (RPM));
  //frc::SmartDashboard::PutBoolean("INTAKE WOULD BE SPINNING", truth);

  //averageRPMs = rollingRPMs(RPM); 
  //frc::SmartDashboard::PutNumber("AvgRPM", (averageRPMs));

  if (frc::DriverStation::IsDisabled()==true && frc::DriverStation::IsAutonomous()==false) {
      switch (AllianceColor) {
      case frc::DriverStation::kRed:
            PutLED(0.59);
          break;
      case frc::DriverStation::kBlue:
            PutLED(0.85);
          break;
      case frc::DriverStation::kInvalid:
      default:
            PutLED(0.00);
          break;
        }
  }

    if (Xbox.GetXButtonReleased() && Xbox.GetPOV()==180){
        if (firstPass == true) {
            Shooter.Config_kF(0, 0.0092, 10);
            Shooter.Config_kP(0, 0.081, 10);
            Shooter.Config_kD(0, 3.5, 10);  
            firstPass = false;
        } else {
            Shooter.Config_kF(0, kShootF, 10);
            Shooter.Config_kP(0, kShootP, 10);
            Shooter.Config_kD(0, kShootD, 10); 
            firstPass = true;  
        }
    }
}

void CargoSubsystem::PutLED(double ledMotorValue) {
    //I WOULD create an enum for this, except that the list of values would take me all day, and is not worth typing out
    //PLUS you'd have to find the table anyways to know how to enter the values
    led_lights.Set(ledMotorValue);
}

bool CargoSubsystem::TopLaserGet() {
    return TopLaserState;
}

bool CargoSubsystem::BottomLaserGet() {
    return BottomLaserState;
}

void CargoSubsystem::IntakeDown() {
    IntakeArm.Set(ControlMode::MotionMagic, 1700); 
}

void CargoSubsystem::GrabBalls() {
    if (TopLaserState) {
        TopIndexer.Set(kIndexerSpeed);
        BottomIndexer.Set(kIndexerSpeed);
        IntakeWheels.Set(kIntakeSpeed);
    } else if (ballCorrectColor) {
        TopIndexer.StopMotor();
        if (BottomLaserState)
        {
            BottomIndexer.Set(kIndexerSpeed);
            IntakeWheels.Set(kIntakeSpeed);
        } else {
            BottomIndexer.StopMotor();
            IntakeWheels.StopMotor();
        }
    } else if (!ballCorrectColor) {
        Shooter.Set(ControlMode::Velocity, -10000);
        TopIndexer.Set(kIndexerSpeed);
        if (BottomLaserState)
        {
            BottomIndexer.Set(kIndexerSpeed);
            IntakeWheels.Set(kIntakeSpeed);
        } else {
            BottomIndexer.StopMotor();
            IntakeWheels.StopMotor();
        }
    }
}

void CargoSubsystem::AutoGrabBalls() {
    if (TopLaserState) {
        TopIndexer.Set(kIndexerSpeed);
        BottomIndexer.Set(kIndexerSpeed);
        IntakeWheels.Set(kIntakeSpeed);
    } else {
        TopIndexer.StopMotor();
        if (BottomLaserState)
        {
            BottomIndexer.Set(kIndexerSpeed);
            IntakeWheels.Set(kIntakeSpeed);
        } else {
            Shooter.Set(ControlMode::Velocity, -39000);
            BottomIndexer.StopMotor();
            IntakeWheels.StopMotor();
            IntakeArm.Set(ControlMode::MotionMagic, -75);
        }
    }   
}

void CargoSubsystem::AllOut() {
    TopIndexer.Set(-kIndexerSpeed);
    BottomIndexer.Set(-kIndexerSpeed);
    IntakeWheels.Set(-kIndexerSpeed);
    IntakeArm.Set(ControlMode::MotionMagic, -75);
}

void CargoSubsystem::Shoot() {
    Shooter.Set(ControlMode::Velocity, -39100);
    IntakeArm.Set(ControlMode::MotionMagic, -75);
    if (TopLaserState) {
        TopIndexer.Set(kIndexerSpeed-0.07);
        BottomIndexer.Set(kIndexerSpeed-0.07);
    } else if (abs(Shooter.GetSelectedSensorVelocity(0)) >= 39000 && abs(Shooter.GetSelectedSensorVelocity(0)) <= 49150) {
        truth = true;
        BottomIndexer.Set(kIndexerSpeed);
        TopIndexer.Set(kIndexerSpeed);
    } else {
        truth = 0;
        BottomIndexer.StopMotor();
        TopIndexer.StopMotor();
        }
}

void CargoSubsystem::AutoShoot() {
    Shooter.Set(ControlMode::Velocity, -39130);
    if (TopLaserState) {
        TopIndexer.Set(kIndexerSpeed);
        BottomIndexer.Set(kIndexerSpeed);
    } else if (abs(Shooter.GetSelectedSensorVelocity(0)) >= 39000 && abs(Shooter.GetSelectedSensorVelocity(0)) <= 49150) {
        truth = true;
        BottomIndexer.Set(kIndexerSpeed);
        TopIndexer.Set(kIndexerSpeed);
    } else {
        truth = 0;
        BottomIndexer.StopMotor();
        TopIndexer.StopMotor();
        }
}

void CargoSubsystem::LowShoot() {
    Shooter.Set(ControlMode::Velocity, -20100);
    if (TopLaserState) {
        TopIndexer.Set(kIndexerSpeed);
        BottomIndexer.Set(kIndexerSpeed);
    } else if (abs(Shooter.GetSelectedSensorVelocity(0)) >= 19800) {
        truth = true;
        BottomIndexer.Set(kIndexerSpeed);
        TopIndexer.Set(kIndexerSpeed);
    } else {
        truth = 0;
        BottomIndexer.StopMotor();
        TopIndexer.StopMotor();
        }
}

void CargoSubsystem::Stop() {
    IntakeWheels.StopMotor();
    BottomIndexer.StopMotor();
    TopIndexer.StopMotor();
    Shooter.Set(ControlMode::PercentOutput, 0.0); 
    IntakeArm.Set(ControlMode::MotionMagic, -75); 
    if (IntakeArm.GetSensorCollection().IsRevLimitSwitchClosed()) {
        IntakeArm.SetSelectedSensorPosition(0, 0, 10);
        IntakeArm.Set(ControlMode::PercentOutput, 0.0);
    } else if ((IntakeArm.GetSensorCollection().IsRevLimitSwitchClosed()==false) && (abs(IntakeArm.GetClosedLoopError(0)) <= 100)) {
        IntakeArm.Set(ControlMode::PercentOutput, -0.25);
    }
}

double CargoSubsystem::GetRPM() {
    return RPM;
}