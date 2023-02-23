// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "RobotContainer.hpp"

#include <iostream>

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <wpi/json.h>

#include "Constants.hpp"
#include "commands/DriveTrajectory.hpp"
#include "commands/ResetAbsoluteEncoders.hpp"
#include "commands/autos/North2ConeChgstat.hpp"
#include "commands/autos/OneConeChgstat.hpp"
#include "commands/autos/South2Cone.hpp"
#include "frc2/command/Command.h"
#include "util/log/DoubleTelemetryEntry.hpp"

using namespace frc;
using namespace frc2;
using namespace OIConstants;
using namespace units;

RobotContainer::RobotContainer()
    : m_north2ConeChgstat(&m_drive, &m_superstructure, &m_trajManager),
      m_south2Cone(&m_drive, &m_superstructure, &m_trajManager),
      m_mid1ConeChgstat(&m_drive, &m_superstructure, &m_trajManager),
      m_oiDriverLeftXLog("OI/Driver/Left X"),
      m_oiDriverRightXLog("OI/Driver/Right X"),
      m_oiDriverRightYLog("OI/Driver/Right Y"),
      m_autoSwitchIndexLog("Auto Switch Index") {
  m_drive.SetDefaultCommand(RunCommand(
      [this] {  // onExecute
        // Right stick up on xbox is negative, right stick down is postive.
        // Right stick right on xbox is negative, right stick left is postive.
        // Left stick right is positive, left stick left is negative.
        double rightXAxis = -m_driver.GetRawAxis(kZorroRightYAxis);
        double rightYAxis = -m_driver.GetRawAxis(kZorroRightXAxis);
        double leftXAxis = -m_driver.GetRawAxis(kZorroLeftXAxis);
        return m_drive.JoystickDrive(
            std::abs(rightXAxis) < 0.05 ? 0.0 : rightXAxis,
            std::abs(rightYAxis) < 0.05 ? 0.0 : rightYAxis,
            std::abs(leftXAxis) < 0.05 ? 0.0 : leftXAxis, true);
      },
      {&m_drive}  // requirements
      ));

  ConfigureBindings();

  SmartDashboard::PutData("Reset Encoders",
                          new ResetAbsoluteEncoders(&m_drive));

  // Initialize the LEDs
  m_leds.SetLength(kLEDBuffLength);
  m_leds.SetData(m_ledBuffer);
  m_leds.Start();
}

std::optional<Command*> RobotContainer::GetAutonomousCommand() {
  UpdateAutoSelected();
  switch (m_currentSelectedAuto) {
    case SelectedAuto::kNorth2ConeChgstat:
      return &m_north2ConeChgstat;
    case SelectedAuto::kSouth2Cone:
      return &m_south2Cone;
    case SelectedAuto::kMid1ConeChgstat:
      return &m_mid1ConeChgstat;
    default:
      return std::nullopt;
  }
}

void RobotContainer::UpdateTelemetry() {
  m_oiDriverLeftXLog.Append(m_driver.GetRawAxis(kZorroLeftXAxis));
  m_oiDriverRightXLog.Append(m_driver.GetRawAxis(kZorroRightXAxis));
  m_oiDriverRightYLog.Append(m_driver.GetRawAxis(kZorroRightYAxis));
  m_compressor.IsEnabled();
  SmartDashboard::PutNumber("Arm absolute position",
                            m_superstructure.GetAbsoluteArmPosition().value());
  // SmartDashboard::PutNumber("String position", m_superstructure.RawString());
  // SmartDashboard::PutNumber("Estimated string position",
  //                           m_superstructure.GetAbsoluteStringPosition());
  // SmartDashboard::PutNumber("Estimated angle",
  //                           m_superstructure.GetStringAngle().value());
  SmartDashboard::PutNumber("Relative angle",
                            m_superstructure.GetRelativePosition());
  SmartDashboard::PutNumber("Raw relative angle",
                            m_superstructure.RawPosition().value());
  
  auto previousAutoSelected = m_currentSelectedAuto;
  UpdateAutoSelected();
  if (previousAutoSelected != m_currentSelectedAuto) {
    m_autoSwitchIndexLog.Append(static_cast<int>(m_currentSelectedAuto));
  }
}

void RobotContainer::ConfigureBindings() {
  JoystickButton zorroGIn{&m_driver, kZorroGIn};
  zorroGIn.OnTrue(InstantCommand([this]() {
                    return m_drive.ResetOdometry(Pose2d());
                  }).ToPtr());

  JoystickButton zorroLeftBumper{&m_driver, kZorroAIn};

  zorroLeftBumper.OnTrue((InstantCommand([this]() {
                           return m_superstructure.Outtake();
                         })).ToPtr());
  zorroLeftBumper.OnFalse((InstantCommand([this]() {
                            return m_superstructure.SetIntakeWheelSpeed(0.0);
                          })).ToPtr());

  // m_operator.X().OnTrue(IntakeCone(&m_gripper).ToPtr());
  // m_operator.X().OnFalse(
  //     InstantCommand([this]() { return m_gripper.Retract(); },
  //     {&m_gripper}).ToPtr());
  // m_operator.Y().OnTrue(IntakeCube(&m_gripper).ToPtr());
  // m_operator.Y().OnFalse(
  //     InstantCommand([this]() { return m_gripper.Retract(); },
  //     {&m_gripper}).ToPtr());
  // m_operator.A().OnTrue(
  //     InstantCommand([this]() { return m_gripper.EjectGamePiece(); },
  //     {&m_gripper}).ToPtr());
  // m_operator.A().OnFalse(
  //     InstantCommand([this]() { return m_gripper.Retract(); },
  //     {&m_gripper}).ToPtr());
  m_operator.X().OnTrue(
      (InstantCommand([this]() { m_superstructure.IntakeCone(); })).ToPtr());
  m_operator.X().OnFalse((InstantCommand([this]() {
                           m_superstructure.SetIntakeWheelSpeed(0.0);
                         })).ToPtr());
  m_operator.Y().OnTrue(
      (InstantCommand([this]() { m_superstructure.IntakeCube(); })).ToPtr());
  m_operator.Y().OnFalse((InstantCommand([this]() {
                           m_superstructure.SetIntakeWheelSpeed(0.0);
                         })).ToPtr());
  m_operator.A().OnTrue((InstantCommand([this]() {
                          m_superstructure.PositionMedium();
                        })).ToPtr());
  m_operator.B().OnTrue(
      (InstantCommand([this]() { m_superstructure.PositionHigh(); })).ToPtr());
  m_operator.LeftBumper().OnTrue((InstantCommand([this]() {
                                   m_superstructure.IntakeCubeStation();
                                 })).ToPtr());
  m_operator.LeftBumper().OnFalse((InstantCommand([this]() {
                                    return m_superstructure.IntakeCube();
                                  })).ToPtr());

  m_operator.RightBumper().OnTrue((InstantCommand([this]() {
                                    m_superstructure.IntakeConeStation();
                                  })).ToPtr());
  m_operator.RightBumper().OnFalse((InstantCommand([this]() {
                                     return m_superstructure.IntakeCone();
                                   })).ToPtr());

  JoystickButton driverRightTrigger(&m_driver, OIConstants::kZorroDIn);
  JoystickButton driverRightBottomTrigger(&m_driver, OIConstants::kZorroHIn);
  driverRightBottomTrigger.OnTrue(InstantCommand([this]() {
                                    m_superstructure.m_flipConeMode = true;
                                  }).ToPtr());
  driverRightBottomTrigger.OnFalse(InstantCommand([this]() {
                                     m_superstructure.m_flipConeMode = false;
                                   }).ToPtr());
  driverRightTrigger.OnTrue(InstantCommand([this]() {
                              m_superstructure.m_flipConeUp = true;
                            }).ToPtr());
  driverRightTrigger.OnFalse(InstantCommand([this]() {
                               m_superstructure.m_flipConeUp = false;
                             }).ToPtr());
  // m_operator.().OnTrue((InstantCommand([this]() { return
  // m_superstructure.SetIntakeWheelSpeed(0.5); })).ToPtr());
  // m_operator.RightBumper().OnFalse((InstantCommand([this]() { return
  // m_superstructure.SetIntakeWheelSpeed(0.0); })).ToPtr());
}

void RobotContainer::RunDisabled() {
  m_superstructure.SyncEncoders();
}

void RobotContainer::SuperstructurePeriodic() {
  m_superstructure.SuperstructurePeriodic();
}

void RobotContainer::LED() {
  if (m_superstructure.HasGamePiece()) {
    GamePieceLED();
  } else {
    if (!m_superstructure.m_expanded) {
      Purple();
    } else {
      Yellow();
    }
  }
  // Rainbow();
  m_leds.SetData(m_ledBuffer);
}

std::optional<size_t> RobotContainer::GetAutoSwitchIndex() const {
  for (size_t switchIndex = 0; switchIndex < ElectricalConstants::kAutoSwitchPorts.size(); switchIndex++) {
    if (!m_autoSwitch[switchIndex].Get()) {
      return switchIndex;
    }
  }
  return std::nullopt;
}

void RobotContainer::UpdateAutoSelected() {
  auto selectionOpt = GetAutoSwitchIndex();
  // std::optional<size_t> selectionOpt = static_cast<size_t>(SmartDashboard::GetNumber("Selected Auto", 0.0));
  SelectedAuto newSelectedAuto;
  using enum SelectedAuto;
  if (!selectionOpt) {
    newSelectedAuto = kNoAuto;
  }
  switch (*selectionOpt) {
    case static_cast<long long>(kNoAuto):
      newSelectedAuto = kNoAuto;
      break;
    case static_cast<long long>(kNorth2ConeChgstat):
      newSelectedAuto = kNorth2ConeChgstat;
      break;
    case static_cast<long long>(kSouth2Cone):
      newSelectedAuto = kSouth2Cone;
      break;
    case static_cast<long long>(kMid1ConeChgstat):
      newSelectedAuto = kMid1ConeChgstat;
      break;
    default:
      newSelectedAuto = kNoAuto;
      break;
  }

  m_currentSelectedAuto = newSelectedAuto;
}

void RobotContainer::GamePieceLED() {
  // For every pixel
  int stripLength = 3;
  int initial = 0;
  bool yellow = true;
  for (int i = 0; i < kLEDBuffLength; i++) {
    if (i - initial > stripLength - 1) {
      initial = i;
      yellow = !yellow;
    }
    if (yellow) {
      m_ledBuffer[i].SetRGB(255, 100, 0);
    } else {
      m_ledBuffer[i].SetRGB(150, 0, 255);
    }
  }
}

void RobotContainer::Yellow() {
  for (int i = 0; i < kLEDBuffLength; i++) {
    m_ledBuffer[i].SetRGB(255, 100, 0);
  }
}

void RobotContainer::Green() {
  for (int i = 0; i < kLEDBuffLength; i++) {
    m_ledBuffer[i].SetRGB(0, 255, 0);
  }
}

void RobotContainer::Purple() {
  for (int i = 0; i < kLEDBuffLength; i++) {
    m_ledBuffer[i].SetRGB(150, 0, 255);
  }
}
