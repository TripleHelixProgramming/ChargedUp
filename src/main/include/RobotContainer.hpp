// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <optional>

#include <frc/Joystick.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.hpp"
#include "frc/PneumaticsBase.h"
#include "subsystems/Superstructure.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "util/TrajectoryManager.hpp"
#include "util/log/DoubleTelemetryEntry.hpp"
#include <frc/Compressor.h>
#include <frc/PneumaticsModuleType.h>

class RobotContainer {
 public:
  RobotContainer();

  std::optional<frc2::CommandPtr> GetAutonomousCommand();

  void UpdateTelemetry();

  void RunDisabled();

  void SuperstructurePeriodic();

 private:
  // Subsystems
  SwerveDrive m_drive;
  Superstructure m_superstructure;
  frc::Compressor m_compressor{1, frc::PneumaticsModuleType::REVPH};

  // Operator Interface (OI)
  frc::Joystick m_driver{0};
  frc2::CommandXboxController m_operator{1};

  TrajectoryManager m_trajManager;

  void ConfigureBindings();

  DoubleTelemetryEntry m_oiDriverLeftXLog;
  DoubleTelemetryEntry m_oiDriverRightXLog;
  DoubleTelemetryEntry m_oiDriverRightYLog;
};
