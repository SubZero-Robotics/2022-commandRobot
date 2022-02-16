#include "commands/ClimberDown.h"

ClimberDown::ClimberDown(ClimberSubsystem* subsystem, frc::XboxController* controller) 
    : m_climber(subsystem), m_controller(controller)  {
  AddRequirements({subsystem});
}

void ClimberDown::Initialize() {

}

void ClimberDown::Execute() {
  m_climber->Down();
}

void ClimberDown::End(bool interrupted) {
  m_climber->Stop(); //should do this anyways with m_climber.SetDefaultCommand
}

bool ClimberDown::IsFinished() {
  return false;
}