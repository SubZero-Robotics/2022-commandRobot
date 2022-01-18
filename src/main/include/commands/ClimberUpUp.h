#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/ClimberSubsystem.h"

class ClimberUpUp : public frc2::CommandHelper<frc2::CommandBase, ClimberUpUp> {   // this line builds fine
    public: 
     explicit ClimberUpUp(ClimberSubsystem* subsystem, frc::XboxController* controller);

     void Initialize() override;
     
     void Execute() override;
     
     void End(bool interrupted) override;
     
     bool IsFinished() override;

    private: 

     ClimberSubsystem* m_climber;
     frc::XboxController* m_controller;

};