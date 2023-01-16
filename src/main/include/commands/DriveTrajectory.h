#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>

#include "subsystems/SwerveDrive.h"
#include "util/Trajectory.h"

class DriveTrajectory
    : public frc2::CommandHelper<frc2::CommandBase, DriveTrajectory> {
 public:
   DriveTrajectory(SwerveDrive* drive, Trajectory& trajectory);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  SwerveDrive* m_drive;

  Trajectory m_trajectory;

  frc::Timer m_timer;
};
