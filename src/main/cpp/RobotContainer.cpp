// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "RobotContainer.hpp"

#include <functional>
#include <iostream>
#include <optional>
#include <tuple>

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <wpi/json.h>

#include "Constants.hpp"
#include "Robot.hpp"
#include "commands/DriveTrajectory.hpp"
#include "commands/ResetAbsoluteEncoders.hpp"
#include "commands/autos/North2ConeHighChgstat.hpp"
#include "commands/autos/North2ConeHighPick1Cone.hpp"
#include "commands/autos/North3ConeLow.hpp"
#include "commands/autos/South2ConeHigh.hpp"
#include "util/log/DoubleTelemetryEntry.hpp"

using namespace ElectricalConstants;
using namespace frc;
using namespace frc2;
using namespace OIConstants;
using namespace units;

RobotContainer::RobotContainer(std::function<bool(void)> isDisabled)
    : m_isDisabled(isDisabled),
      m_blueNorth2ConeHighChgstat(&m_drive, &m_superstructure, true),
      m_blueNorth2ConeHighPick1Cone(&m_drive, &m_superstructure, true),
      m_blueSouth2ConeHigh(&m_drive, &m_superstructure, true),
      m_blueNorth3ConeLow(&m_drive, &m_superstructure, true),
      m_redNorth2ConeHighChgstat(&m_drive, &m_superstructure, false),
      m_redNorth2ConeHighPick1Cone(&m_drive, &m_superstructure, false),
      m_redSouth2ConeHigh(&m_drive, &m_superstructure, false),
      m_redNorth3ConeLow(&m_drive, &m_superstructure, false),
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
            std::abs(rightXAxis) < 0.025 ? 0.0 : rightXAxis,
            std::abs(rightYAxis) < 0.025 ? 0.0 : rightYAxis,
            std::abs(leftXAxis) < 0.025 ? 0.0 : leftXAxis, true, m_isBlue);
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

  m_lastGamePieceIntake.Start();
}

std::optional<Command*> RobotContainer::GetAutonomousCommand() {
  UpdateAutoSelected();
  UpdateIsBlue();
  switch (m_currentSelectedAuto) {
    case SelectedAuto::kNorth2ConeHighChgstat:
      if (m_isBlue)
        return &m_blueNorth2ConeHighChgstat;
      else
        return &m_redNorth2ConeHighChgstat;
    case SelectedAuto::kSouth2ConeHigh:
      if (m_isBlue)
        return &m_blueSouth2ConeHigh;
      else
        return &m_redSouth2ConeHigh;
    case SelectedAuto::kNorth2ConeHighPick1Cone:
      if (m_isBlue)
        return &m_blueNorth2ConeHighPick1Cone;
      else
        return &m_redNorth2ConeHighPick1Cone;
    case SelectedAuto::kNorth3ConeLow:
      if (m_isBlue)
        return &m_blueNorth3ConeLow;
      else
        return &m_redNorth3ConeLow;
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
  SmartDashboard::PutNumber("Relative angle",
                            m_superstructure.GetRelativePosition());
  SmartDashboard::PutNumber("Raw relative angle",
                            m_superstructure.RawPosition().value());

  auto previousAutoSelected = m_currentSelectedAuto;
  UpdateAutoSelected();
  UpdateIsBlue();
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

  m_operator.X().OnTrue(
      (InstantCommand([this]() { m_superstructure.IntakeCone(); })).ToPtr());
  m_operator.X().OnFalse((frc2::InstantCommand([this]() {
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
  m_operator.A().OnFalse(
      (InstantCommand([this]() { m_superstructure.PositionLow(); })).ToPtr());
  m_operator.B().OnTrue(
      (InstantCommand([this]() { m_superstructure.PositionHigh(); })).ToPtr());
  m_operator.B().OnFalse(
      (InstantCommand([this]() { m_superstructure.PositionLow(); })).ToPtr());
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
  frc2::Trigger operatorPOVUp =
      frc2::Trigger([&]() { return m_operator.GetPOV() == 0; });
  operatorPOVUp.OnTrue(InstantCommand([this]() {
                         m_superstructure.m_flipConeMode = true;
                       }).ToPtr());
  operatorPOVUp.OnFalse(InstantCommand([this]() {
                          m_superstructure.m_flipConeMode = false;
                        }).ToPtr());

  JoystickButton operatorView(&m_operator, OIConstants::kXboxView);
  operatorView.OnTrue(
      InstantCommand([this]() { m_superstructure.m_stealth = true; }).ToPtr());
  operatorView.OnFalse(
      InstantCommand([this]() { m_superstructure.m_stealth = false; }).ToPtr());
}

void RobotContainer::RunDisabled() {
  m_drive.SyncAbsoluteEncoders();
  m_superstructure.SyncEncoders();
}

void RobotContainer::SuperstructurePeriodic() {
  m_superstructure.SuperstructurePeriodic();
}

void RobotContainer::LED() {
  if (m_superstructure.HasGamePiece() && !m_lastIntake) {
    m_lastGamePieceIntake.Reset();
    m_lastIntake = true;
  }
  if (!m_superstructure.HasGamePiece()) {
    m_lastIntake = false;
  }

  if (m_isDisabled()) {
    ClearLED();
    // ApplyLEDSingleStrip({std::tuple{255, 255, 255}, {100, 0, 0}, {100, 0, 0},
    // {100, 0, 0}});
    AutoLED();
    // SnakeBOI();
  } else if (m_superstructure.HasGamePiece()) {
    if (m_lastGamePieceIntake.HasElapsed(1_s)) {
      GamePieceLED();
    } else {
      Green();
    }
  } else {
    if (!m_superstructure.m_expanded) {
      Purple();
    } else {
      Yellow();
    }
  }
  m_leds.SetData(m_ledBuffer);
}

std::optional<size_t> RobotContainer::GetAutoSwitchIndex() const {
  for (size_t switchIndex = 0;
       switchIndex < ElectricalConstants::kAutoSwitchPorts.size();
       switchIndex++) {
    if (!m_autoSwitch.at(switchIndex).Get()) {
      SmartDashboard::PutNumber("Auto Rotary Switch Index", switchIndex);
      return switchIndex;
    }
  }
  return std::nullopt;
}

void RobotContainer::UpdateAutoSelected() {
  auto selectionOpt = GetAutoSwitchIndex();
  // std::optional<size_t> selectionOpt =
  // static_cast<size_t>(SmartDashboard::GetNumber("Selected Auto", 0.0));
  using enum SelectedAuto;
  if (!selectionOpt) {
    m_currentSelectedAuto = kNoAuto;
    return;
  }
  int64_t switchIdx = *selectionOpt;
  constexpr auto kMaxAutoIdx = static_cast<int64_t>(kNumberOfAutos) - 1;
  if (switchIdx >= 0 && switchIdx <= kMaxAutoIdx) {
    m_currentSelectedAuto = static_cast<SelectedAuto>(switchIdx);
    return;
  }
  m_currentSelectedAuto = kNoAuto;
}

void RobotContainer::UpdateIsBlue() {
  m_isBlue = !m_redBlueSwitch.Get();
}

void RobotContainer::ApplyLEDSingleStrip(
    const std::array<std::tuple<int, int, int>, kLEDStripLength>& stripBuffer,
    int stripID) {
  // our strips are weirdly wired, sorry

  static constexpr auto kStripLen = kLEDStripLength;
  size_t firstLEDIdx = stripID * kStripLen;
  bool stripDirection = kStripDirections.at(stripID);
  for (size_t stripBuffIdx = 0; stripBuffIdx < kStripLen; stripBuffIdx++) {
    if (stripDirection) {  // if goes down
      m_ledBuffer.at(firstLEDIdx + stripBuffIdx)
          .SetRGB(std::get<0>(stripBuffer.at(kStripLen - 1 - stripBuffIdx)),
                  std::get<1>(stripBuffer.at(kStripLen - 1 - stripBuffIdx)),
                  std::get<2>(stripBuffer.at(kStripLen - 1 - stripBuffIdx)));
    } else {  // if goes up
      m_ledBuffer.at(firstLEDIdx + stripBuffIdx)
          .SetRGB(std::get<0>(stripBuffer.at(stripBuffIdx)),
                  std::get<1>(stripBuffer.at(stripBuffIdx)),
                  std::get<2>(stripBuffer.at(stripBuffIdx)));
    }
  }
}

void RobotContainer::ClearLED() {
  for (size_t clrIdx = 0; clrIdx < kLEDBuffLength; clrIdx++) {
    m_ledBuffer.at(clrIdx).SetRGB(0, 0, 0);
  }
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
      m_ledBuffer.at(i).SetRGB(255, 100, 0);
    } else {
      m_ledBuffer.at(i).SetRGB(150, 0, 255);
    }
  }
}

void RobotContainer::Yellow() {
  for (int i = 0; i < kLEDBuffLength; i++) {
    m_ledBuffer.at(i).SetRGB(255, 100, 0);
  }
}

void RobotContainer::Green() {
  for (int i = 0; i < kLEDBuffLength; i++) {
    m_ledBuffer.at(i).SetRGB(0, 255, 0);
  }
}

void RobotContainer::Purple() {
  for (int i = 0; i < kLEDBuffLength; i++) {
    m_ledBuffer.at(i).SetRGB(150, 0, 255);
  }
}

void RobotContainer::SnakeBOI() {
  ClearLED();
  if (m_previousSnakeIndex == kLEDBuffLength - 1) {
    m_previousSnakeIndex = 0;
  } else {
    m_previousSnakeIndex++;
  }
  m_ledBuffer.at(m_previousSnakeIndex).SetRGB(255, 0, 0);
}

template <typename TE, typename TT>
int _threeWayError(TE error, TT errorTol) {
  if (units::math::abs(error) > errorTol) {
    if (error.value() > 0) {
      return +1;
    } else {
      return -1;
    }
  } else {
    return 0;
  }
}

/**
 * @brief Helper function to calculate the robot-relative dimensions and
 * directions the robot deviates from a target pose. For example, if the robot
 * is too far the right (+x) and too far up (+y) relative to the robot
 * coordinate system and within the range for rotation, the result will be {1,
 * 1, 0}.
 *
 * @return {x-diff, y-diff, theta-diff} (diff ∈ {-1, 0, 1})
 */
std::tuple<int, int, int> _poseWithin(Pose2d target, Pose2d actual) {
  Pose2d transformedActual = actual.RelativeTo(target);
  auto diff = Pose2d() - transformedActual;
  SmartDashboard::PutNumber("Auto Error/X (m)", diff.X().value());
  SmartDashboard::PutNumber("Auto Error/Y (m)", diff.Y().value());
  SmartDashboard::PutNumber("Auto Error/Theta (deg)",
                            diff.Rotation().Degrees().value());
  return {_threeWayError(diff.X(), 3_in), _threeWayError(diff.Y(), 3_in),
          _threeWayError(diff.Rotation().Degrees(), 2_deg)};
}

void RobotContainer::AutoLED() {
  auto selectedAutoID = static_cast<size_t>(m_currentSelectedAuto);
  std::array<std::array<std::tuple<int, int, int>, kLEDStripLength>, 4>
      stripBuffers;
  // show which auto is selected
  for (size_t selectedAutoIdx = 0; selectedAutoIdx < selectedAutoID;
       selectedAutoIdx++) {
    for (size_t chunkIdx = 0; chunkIdx < 2; chunkIdx++) {
      for (size_t stripIdx = 0; stripIdx < 4; stripIdx++) {
        stripBuffers.at(stripIdx).at(selectedAutoIdx * 3 + chunkIdx) = {
            m_isBlue ? 0 : 255, 0, m_isBlue ? 255 : 0};
      }
    }
  }
  // check if robot is staged in correct location
  auto currentPose = m_drive.GetPose();
  // -2 makes sure lights aren't green if no auto selected
  std::tuple<int, int, int> errors{-2, -2, -2};
  switch (m_currentSelectedAuto) {
    case SelectedAuto::kNorth2ConeHighChgstat:
      errors = _poseWithin(currentPose,
                           North2ConeHighChgstat::GetStartingPose(m_isBlue));
      break;
    case SelectedAuto::kSouth2ConeHigh:
      errors =
          _poseWithin(currentPose, South2ConeHigh::GetStartingPose(m_isBlue));
      break;
    case SelectedAuto::kNorth2ConeHighPick1Cone:
      errors = _poseWithin(currentPose,
                           North2ConeHighPick1Cone::GetStartingPose(m_isBlue));
      break;
    case SelectedAuto::kNorth3ConeLow:
      errors =
          _poseWithin(currentPose, North3ConeLow::GetStartingPose(m_isBlue));
      break;
    default:
      break;
  }
  if (std::get<0>(errors) == 0 &&
      std::get<1>(errors) == 0) {  // if translation is good
    for (size_t goodPoseIdx = kLEDStripLength - 1;
         goodPoseIdx >= kLEDStripLength - 1 - 4; goodPoseIdx--) {
      for (size_t stripIdx = 0; stripIdx < 4; stripIdx++) {
        stripBuffers.at(stripIdx).at(goodPoseIdx) = {0, 255, 0};
      }
    }
  }
  if (std::get<2>(errors) == 0) {  // if rotation is good
    for (size_t goodPoseIdx = kLEDStripLength - 1 - 3;
         goodPoseIdx >= kLEDStripLength - 1 - 3 - 2; goodPoseIdx--) {
      for (size_t stripIdx = 0; stripIdx < 4; stripIdx++) {
        stripBuffers.at(stripIdx).at(goodPoseIdx) = {0, 255, 0};
      }
    }
  }
  if (std::get<0>(errors) == +1) {  // if too far right
    for (size_t badPoseIdx = kLEDStripLength - 1;
         badPoseIdx >= kLEDStripLength - 1 - 2; badPoseIdx--) {
      stripBuffers.at(1).at(badPoseIdx) = {255, 0, 0};
      stripBuffers.at(2).at(badPoseIdx) = {255, 0, 0};
    }
  }
  if (std::get<0>(errors) == -1) {  // if too far left
    for (size_t badPoseIdx = kLEDStripLength - 1;
         badPoseIdx >= kLEDStripLength - 1 - 2; badPoseIdx--) {
      stripBuffers.at(0).at(badPoseIdx) = {255, 0, 0};
      stripBuffers.at(3).at(badPoseIdx) = {255, 0, 0};
    }
  }
  if (std::get<1>(errors) == +1) {  // if too far up
    for (size_t badPoseIdx = kLEDStripLength - 1;
         badPoseIdx >= kLEDStripLength - 1 - 2; badPoseIdx--) {
      stripBuffers.at(2).at(badPoseIdx) = {255, 0, 0};
      stripBuffers.at(3).at(badPoseIdx) = {255, 0, 0};
    }
  }
  if (std::get<1>(errors) == -1) {  // if too far down
    for (size_t badPoseIdx = kLEDStripLength - 1;
         badPoseIdx >= kLEDStripLength - 1 - 2; badPoseIdx--) {
      stripBuffers.at(0).at(badPoseIdx) = {255, 0, 0};
      stripBuffers.at(1).at(badPoseIdx) = {255, 0, 0};
    }
  }
  if (std::get<2>(errors) == +1) {  // if too far counterclockwise
    for (size_t badPoseIdx = kLEDStripLength - 1 - 3;
         badPoseIdx >= kLEDStripLength - 1 - 3 - 2; badPoseIdx--) {
      stripBuffers.at(0).at(badPoseIdx) = {255, 0, 0};
      stripBuffers.at(1).at(badPoseIdx) = {255, 0, 0};
      stripBuffers.at(2).at(badPoseIdx) = {255, 0, 0};
      stripBuffers.at(3).at(badPoseIdx) = {255, 0, 0};
    }
  }
  if (std::get<2>(errors) == -1) {  // if too far clockwise
    for (size_t badPoseIdx = kLEDStripLength - 1 - 3;
         badPoseIdx >= kLEDStripLength - 1 - 3 - 2; badPoseIdx--) {
      stripBuffers.at(0).at(badPoseIdx) = {0, 0, 255};
      stripBuffers.at(1).at(badPoseIdx) = {0, 0, 255};
      stripBuffers.at(2).at(badPoseIdx) = {0, 0, 255};
      stripBuffers.at(3).at(badPoseIdx) = {0, 0, 255};
    }
  }
  // apply to actual LED strips
  for (size_t stripIdx = 0; stripIdx < 4; stripIdx++) {
    ApplyLEDSingleStrip(stripBuffers.at(stripIdx), stripIdx);
  }
}
