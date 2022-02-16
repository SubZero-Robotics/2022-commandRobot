#include "commands/ClimberUp.h"

ClimberUp::ClimberUp(ClimberSubsystem* subsystem, frc::XboxController* controller) 
    : m_climber(subsystem), m_controller(controller)  {
  AddRequirements({subsystem});
}

void ClimberUp::Initialize() {

}

void ClimberUp::Execute() {
  m_climber->Up();
}

void ClimberUp::End(bool interrupted) {
 m_climber->Stop(); //should do this anyways with m_climber.SetDefaultCommand
}

bool ClimberUp::IsFinished() {
  return false;
}