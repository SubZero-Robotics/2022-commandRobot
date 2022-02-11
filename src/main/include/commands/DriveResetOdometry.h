#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/DriveSubsystem.h"

class DriveResetOdometry : public frc2::CommandHelper<frc2::CommandBase, DriveResetOdometry> {   // this line builds fine
    public: 
     explicit DriveResetOdometry(DriveSubsystem* subsystem, frc::XboxController* controller);

     void Initialize() override;
     
     void Execute() override;
     
     void End(bool interrupted) override;
     
     bool IsFinished() override;

    private: 
     DriveSubsystem* m_drive;
     frc::XboxController* m_controller;

};