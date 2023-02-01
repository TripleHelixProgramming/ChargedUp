// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <optional>

#include <frc/Joystick.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/Superstructure.h"
#include "subsystems/SwerveDrive.h"
#include "util/TrajectoryManager.h"
#include "util/log/DoubleTelemetryEntry.h"

class RobotContainer {
 public:
  RobotContainer();

  std::optional<frc2::CommandPtr> GetAutonomousCommand();

  void UpdateTelemetry();

  void ArmPeriodic();

 private:
  // Subsystems
  SwerveDrive m_drive;
  Superstructure m_superstructure;

  // Operator Interface (OI)
  frc::Joystick m_driver{0};
  frc2::CommandXboxController m_operator{1};

  TrajectoryManager m_trajManager;

  void ConfigureBindings();

  DoubleTelemetryEntry m_oiDriverLeftXLog;
  DoubleTelemetryEntry m_oiDriverRightXLog;
  DoubleTelemetryEntry m_oiDriverRightYLog;
};
