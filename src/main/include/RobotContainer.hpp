// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <optional>

#include <frc/AddressableLED.h>
#include <frc/Compressor.h>
#include <frc/Joystick.h>
#include <frc/PneumaticsBase.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.hpp"
#include "subsystems/Superstructure.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "util/TrajectoryManager.hpp"
#include "util/log/DoubleTelemetryEntry.hpp"

class RobotContainer {
 public:
  RobotContainer();

  std::optional<frc2::CommandPtr> GetAutonomousCommand();

  void UpdateTelemetry();

  void RunDisabled();

  void SuperstructurePeriodic();

  void GamePieceLED();
  void Yellow();
  void Green();
  void Purple();

  void LED();

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

  /// LED strip
  static constexpr int kLEDBuffLength = 88;
  frc::AddressableLED m_leds{0};
  std::array<frc::AddressableLED::LEDData, kLEDBuffLength> m_ledBuffer;
  int firstPixelHue;
};
