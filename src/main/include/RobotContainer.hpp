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
#include "commands/autos/North2ConeChgstat.hpp"
#include "commands/autos/OneConeChgstat.hpp"
#include "commands/autos/South2Cone.hpp"
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

 private:
  std::function<bool(void)> m_isDisabled;

  // Subsystems
  SwerveDrive m_drive;
  Superstructure m_superstructure;
  frc::Compressor m_compressor{1, frc::PneumaticsModuleType::REVPH};

  // Operator Interface (OI)
  frc::Joystick m_driver{0};
  frc2::CommandXboxController m_operator{1};

  /**
   * Maps auto routines to auto rotary switch indices
   */
  enum class SelectedAuto {
    kNoAuto = 0,  // In case we can't find an auto that works with our alliance
    kNorth2ConeChgstat = 1,
    kSouth2Cone = 2,
    kMid1ConeChgstat = 3,
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

  TrajectoryManager m_trajManager;

  North2ConeChgstat m_north2ConeChgstat;
  South2Cone m_south2Cone;
  OneConeChgstat m_mid1ConeChgstat;

  void ConfigureBindings();

  DoubleTelemetryEntry m_oiDriverLeftXLog;
  DoubleTelemetryEntry m_oiDriverRightXLog;
  DoubleTelemetryEntry m_oiDriverRightYLog;
  DoubleTelemetryEntry m_autoSwitchIndexLog;

  /// LED strip
  static constexpr int kLEDBuffLength = 88;
  static constexpr bool kStripDirections[] = {
      false, true, true, true};  // false means up, true means down
  frc::AddressableLED m_leds{0};
  std::array<frc::AddressableLED::LEDData, kLEDBuffLength> m_ledBuffer;

  void ApplyLEDSingleStrip(const std::array<std::tuple<int, int, int>,
                                            kLEDBuffLength / 4>& stripBuffer);

  void ClearLED();
  void GamePieceLED();
  void Yellow();
  void Green();
  void Purple();
  int m_previousSnakeIndex = -1;
  /// Used to test the order of the lights
  void SnakeBOI();
  void AutoLED();
};
