/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShooterAutoShoot.h"
#include <frc/smartdashboard/Smartdashboard.h>

#include "commands/TurnToLimelight.h"



ShooterAutoShoot::ShooterAutoShoot(CargoSubsystem* subsystem, frc::XboxController* controller) 
    : m_cargo(subsystem), m_controller(controller)  {
  AddRequirements({subsystem});
}

void ShooterAutoShoot::Initialize() {
  //if top and bottom blocked, we have two balls
  if (TopIntakeLaser.Get()==false && BottomIntakeLaser.Get()==false) {
    CargoToShoot = 2;
  //if only top blocked, we have one ball
  } else if (TopIntakeLaser.Get()==false && BottomIntakeLaser.Get()==true) {
    CargoToShoot = 1;
  //default to two balls
  } else {
    CargoToShoot = 2;
  }
}
 
void ShooterAutoShoot::Execute() {
  // tell shooter to get to set rpm
  m_cargo->AutoShoot();
  //if top index blocked, we are armed to shoot
  if (TopIntakeLaser.Get()==false) {
    armed = true;
  }
  //if top "was" blocked (armed used to equal true) and it's not anymore, we have one less ball
  if (TopIntakeLaser.Get()==true && armed==true) {
    CargoToShoot--;
    armed = false;
  }

}

void ShooterAutoShoot::End(bool interrupted) {
  m_cargo->Stop();
}

// this will stop after we have zero more balls
bool ShooterAutoShoot::IsFinished() { if (CargoToShoot==0) {return true;} else {return false;} }
