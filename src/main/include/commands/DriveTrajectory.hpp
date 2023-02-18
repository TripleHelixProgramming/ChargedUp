// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.hpp"
#include "util/Trajectory.hpp"
#include "util/log/DoubleTelemetryEntry.hpp"

class DriveTrajectory
    : public frc2::CommandHelper<frc2::CommandBase, DriveTrajectory> {
 public:
  DriveTrajectory(SwerveDrive* drive, const Trajectory* trajectory, bool useVision = true);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDrive* m_drive;

  const Trajectory* m_trajectory;

  bool m_useVision;

  frc::Timer m_timestamp;

  frc::PIDController m_controllerX{6.0, 0, 0};
  frc::PIDController m_controllerY{6.0, 0, 0};
  frc::PIDController m_controllerRotation{10.0, 0, 0};

  DoubleTelemetryEntry m_timestampLog;

  DoubleTelemetryEntry m_xSetpointLog;
  DoubleTelemetryEntry m_ySetpointLog;
  DoubleTelemetryEntry m_thetaSetpointLog;
};
