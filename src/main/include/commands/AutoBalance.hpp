// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.hpp"
#include "util/Trajectory.hpp"
#include "util/log/DoubleTelemetryEntry.hpp"

class AutoBalance
    : public frc2::CommandHelper<frc2::CommandBase, AutoBalance> {
 public:
  AutoBalance(SwerveDrive* drive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDrive* m_drive;

  double m_lastAngle = 0.0;
  double m_lastTime = 0.0;
  double m_tippedTime = 0.0;
  double m_startTime = 0.0;

  frc::Timer timer;

  bool tipped = false;
  bool end = false;
};
