/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ShooterAutoShoot.h"
#include <frc/smartdashboard/Smartdashboard.h>
#include <frc2/command/WaitCommand.h>



ShooterAutoShoot::ShooterAutoShoot(CargoSubsystem* subsystem, frc::XboxController* controller) 
    : m_cargo(subsystem), m_controller(controller)  {
  AddRequirements({subsystem});
}

void ShooterAutoShoot::Initialize() {
  //if top and bottom blocked, we have two balls
  if (m_cargo->TopLaserGet()==false && m_cargo->BottomLaserGet()==false) {
    CargoToShoot = 2;
  //if only top blocked, we have one ball
  } else if (m_cargo->TopLaserGet()==false && m_cargo->BottomLaserGet()==true) {
    CargoToShoot = 2;
  //default to two balls
  } else {
    CargoToShoot = 2;
  }
}
 
void ShooterAutoShoot::Execute() {
  // tell shooter to get to set rpm
  m_cargo->AutoShoot();
  //if top index blocked, we are armed to shoot
  if (m_cargo->TopLaserGet()==false) {
    armed = true;
  }
  //if top "was" blocked (armed used to equal true) and it's not anymore, we have one less ball
  if (m_cargo->TopLaserGet()==true && armed==true) {
    CargoToShoot--;
    armed = false;
    m_cargo->PutLED(-0.05);
  }

}

void ShooterAutoShoot::End(bool interrupted) {
  frc2::WaitCommand(0.1_s);
  m_cargo->Stop();
}

// this will stop after we have zero more balls
bool ShooterAutoShoot::IsFinished() { if (CargoToShoot==0) {return true;} else {return false;} }
