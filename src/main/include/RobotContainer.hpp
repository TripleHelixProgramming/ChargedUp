// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <optional>
#include <tuple>

#include <frc/AddressableLED.h>
#include <frc/Compressor.h>
#include <frc/DigitalInput.h>
#include <frc/Joystick.h>
#include <frc/PneumaticsBase.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.hpp"
#include "commands/autos/Mid1ConeChgstat.hpp"
#include "commands/autos/North2ConeChgstat.hpp"
#include "commands/autos/North3Cone.hpp"
#include "commands/autos/North3Cube.hpp"
#include "commands/autos/South2Cone.hpp"
#include "networktables/DoubleTopic.h"
#include "subsystems/Superstructure.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "util/TrajectoryManager.hpp"
#include "util/log/DoubleTelemetryEntry.hpp"

class RobotContainer {
 public:
  explicit RobotContainer(std::function<bool(void)> isDisabled);

  std::optional<frc2::Command*> GetAutonomousCommand();

  void UpdateTelemetry();

  void RunDisabled();

  void SuperstructurePeriodic();

  /**
   * Query the RIO's digital input pins to detect currently selected auto
   * routine index.
   *
   * @return the auto rotary switch index if detected, if not nullopt is
   * returned
   */
  std::optional<size_t> GetAutoSwitchIndex() const;

  void UpdateAutoSelected();
  void UpdateIsBlue();

  void LED();

  nt::DoublePublisher m_publisher;

 private:
  std::function<bool(void)> m_isDisabled;

  // Subsystems
  SwerveDrive m_drive;
  Superstructure m_superstructure;
  frc::Compressor m_compressor{ElectricalConstants::kPHPort,
                               frc::PneumaticsModuleType::REVPH};

  // Operator Interface (OI)
  frc::Joystick m_driver{OIConstants::kDriverControllerPort};
  frc2::CommandXboxController m_operator{OIConstants::kOperatorControllerPort};

  /**
   * Maps auto routines to auto rotary switch indices
   */
  enum class SelectedAuto {
    kNoAuto = 0,  // In case we can't find an auto that works with our alliance
    kNorth2ConeChgstat = 1,
    kSouth2Cone = 2,
    kNorth3Cone = 3,
    kNorth3Cube = 4,
    kMid1ConeChgstat = 5,
    kNumberOfAutos
  };

  SelectedAuto m_currentSelectedAuto = SelectedAuto::kNorth2ConeChgstat;
  bool m_isBlue = true;

  const std::array<frc::DigitalInput, 8> m_autoSwitch = {
      frc::DigitalInput(ElectricalConstants::kAutoSwitchPorts[0]),
      frc::DigitalInput(ElectricalConstants::kAutoSwitchPorts[1]),
      frc::DigitalInput(ElectricalConstants::kAutoSwitchPorts[2]),
      frc::DigitalInput(ElectricalConstants::kAutoSwitchPorts[3]),
      frc::DigitalInput(ElectricalConstants::kAutoSwitchPorts[4]),
      frc::DigitalInput(ElectricalConstants::kAutoSwitchPorts[5]),
      frc::DigitalInput(ElectricalConstants::kAutoSwitchPorts[6]),
      frc::DigitalInput(ElectricalConstants::kAutoSwitchPorts[7])};

  const frc::DigitalInput m_redBlueSwitch{
      ElectricalConstants::kRedBlueSwitchPort};

  North2ConeChgstat m_blueNorth2ConeChgstat;
  South2Cone m_blueSouth2Cone;
  North3Cone m_blueNorth3Cone;
  North3Cube m_blueNorth3Cube;
  Mid1ConeChgstat m_blueMid1ConeChgstat;

  North2ConeChgstat m_redNorth2ConeChgstat;
  South2Cone m_redSouth2Cone;
  North3Cone m_redNorth3Cone;
  North3Cube m_redNorth3Cube;
  Mid1ConeChgstat m_redMid1ConeChgstat;

  void ConfigureBindings();

  DoubleTelemetryEntry m_oiDriverLeftXLog;
  DoubleTelemetryEntry m_oiDriverRightXLog;
  DoubleTelemetryEntry m_oiDriverRightYLog;
  DoubleTelemetryEntry m_autoSwitchIndexLog;

  frc::AddressableLED m_leds{0};
  std::array<frc::AddressableLED::LEDData, ElectricalConstants::kLEDBuffLength>
      m_ledBuffer;

  void ApplyLEDSingleStrip(
      const std::array<std::tuple<int, int, int>,
                       ElectricalConstants::kLEDStripLength>& stripBuffer,
      int stripID);

  void ClearLED();
  void GamePieceLED();
  void Yellow();
  void Green();
  void Purple();
  int m_previousSnakeIndex = -1;
  /// Used to test the order of the lights
  void SnakeBOI();
  void AutoLED();

 private:
  frc::Timer m_lastGamePieceIntake;
  bool m_lastIntake = false;
};
