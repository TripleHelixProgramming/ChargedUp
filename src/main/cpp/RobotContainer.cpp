// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "RobotContainer.hpp"

#include <functional>
#include <iostream>
#include <optional>
#include <tuple>

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>
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
#include "commands/autos/Mid1ConeChgstat.hpp"
#include "commands/autos/North2ConeChgstat.hpp"
#include "commands/autos/North3Cone.hpp"
#include "commands/autos/South2Cone.hpp"
#include "util/log/DoubleTelemetryEntry.hpp"

using namespace ElectricalConstants;
using namespace frc;
using namespace frc2;
using namespace OIConstants;
using namespace units;

RobotContainer::RobotContainer(std::function<bool(void)> isDisabled)
    : m_isDisabled(isDisabled),
      m_blueNorth2ConeChgstat(&m_drive, &m_superstructure, true),
      m_blueSouth2Cone(&m_drive, &m_superstructure, true),
      m_blueNorth3Cone(&m_drive, &m_superstructure, true),
      m_blueMid1ConeChgstat(&m_drive, &m_superstructure, true),
      m_redNorth2ConeChgstat(&m_drive, &m_superstructure, false),
      m_redSouth2Cone(&m_drive, &m_superstructure, false),
      m_redNorth3Cone(&m_drive, &m_superstructure, false),
      m_redMid1ConeChgstat(&m_drive, &m_superstructure, false),
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
    case SelectedAuto::kNorth2ConeChgstat:
      if (m_isBlue)
        return &m_blueNorth2ConeChgstat;
      else
        return &m_redNorth2ConeChgstat;
    case SelectedAuto::kSouth2Cone:
      if (m_isBlue)
        return &m_blueSouth2Cone;
      else
        return &m_redSouth2Cone;
    case SelectedAuto::kNorth3Cone:
      if (m_isBlue)
        return &m_blueNorth3Cone;
      else
        return &m_redNorth3Cone;
    case SelectedAuto::kMid1ConeChgstat:
      if (m_isBlue)
        return &m_blueMid1ConeChgstat;
      else
        return &m_redMid1ConeChgstat;
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
  m_operator.X().OnFalse((InstantCommand([this]() {
                           m_superstructure.SetIntakeWheelSpeed(0.0);
                         })).ToPtr());
  m_operator.Y().OnTrue(
      (InstantCommand([this]() { m_superstructure.IntakeCube(); })).ToPtr());
  m_operator.Y().OnFalse((InstantCommand([this]() {
                           m_superstructure.SetIntakeWheelSpeed(0.0);
                         })).ToPtr());
  m_operator.A().WhileTrue(
      (RunCommand([this]() { m_superstructure.PositionMedium(); })).ToPtr());
  m_operator.A().OnFalse(
      (InstantCommand([this]() { m_superstructure.PositionLow(); })).ToPtr());
  m_operator.B().WhileTrue(
      (RunCommand([this]() { m_superstructure.PositionHigh(); })).ToPtr());
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

  // m_operator.().OnTrue((InstantCommand([this]() { return
  // m_superstructure.SetIntakeWheelSpeed(0.5); })).ToPtr());
  // m_operator.RightBumper().OnFalse((InstantCommand([this]() { return
  // m_superstructure.SetIntakeWheelSpeed(0.0); })).ToPtr());
}

void RobotContainer::RunDisabled() {
  m_drive.SyncAbsoluteEncoders();
  m_superstructure.SyncEncoders();

  // RED:
  // auto 1 and 3 use left cam
  // auto 2 use right cam
  // BLUE:
  // auto 1 and 3 use right cam
  // auto 2 use left cam

  bool useLeftCam;

  switch (m_currentSelectedAuto) {
    case SelectedAuto::kNorth2ConeChgstat:
    case SelectedAuto::kNorth3Cone:
    case SelectedAuto::kNorth3Cube:
    default:
      useLeftCam = !m_isBlue;
      break;
    case SelectedAuto::kSouth2Cone:
      useLeftCam = m_isBlue;
      break;
  }
  SmartDashboard::PutBoolean("Vision/Using Left Cam", useLeftCam);
  m_drive.SetVisionUsingLeftCam(useLeftCam);
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
    if (!m_autoSwitch[switchIndex].Get()) {
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
  SelectedAuto newSelectedAuto;
  using enum SelectedAuto;
  if (!selectionOpt) {
    newSelectedAuto = kNoAuto;
  }
  switch (*selectionOpt) {
    case static_cast<int64_t>(kNoAuto):
      newSelectedAuto = kNoAuto;
      break;
    case static_cast<int64_t>(kNorth2ConeChgstat):
      newSelectedAuto = kNorth2ConeChgstat;
      break;
    case static_cast<int64_t>(kSouth2Cone):
      newSelectedAuto = kSouth2Cone;
      break;
    case static_cast<int64_t>(kNorth3Cone):
      newSelectedAuto = kNorth3Cone;
      break;
    case static_cast<int64_t>(kNorth3Cube):
      newSelectedAuto = kNorth3Cube;
      break;
    case static_cast<int64_t>(kMid1ConeChgstat):
      newSelectedAuto = kMid1ConeChgstat;
      break;
    case static_cast<int64_t>(kWeBeGaming):
      newSelectedAuto = kWeBeGaming;
      break;
    default:
      newSelectedAuto = kNoAuto;
      break;
  }

  m_currentSelectedAuto = newSelectedAuto;
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
  bool stripDirection = kStripDirections[stripID];
  for (size_t stripBuffIdx = 0; stripBuffIdx < kStripLen; stripBuffIdx++) {
    if (stripDirection) {  // if goes down
      m_ledBuffer[firstLEDIdx + stripBuffIdx].SetRGB(
          std::get<0>(stripBuffer[kStripLen - 1 - stripBuffIdx]),
          std::get<1>(stripBuffer[kStripLen - 1 - stripBuffIdx]),
          std::get<2>(stripBuffer[kStripLen - 1 - stripBuffIdx]));
    } else {  // if goes up
      m_ledBuffer[firstLEDIdx + stripBuffIdx].SetRGB(
          std::get<0>(stripBuffer[stripBuffIdx]),
          std::get<1>(stripBuffer[stripBuffIdx]),
          std::get<2>(stripBuffer[stripBuffIdx]));
    }
  }
}

void RobotContainer::ClearLED() {
  for (size_t clrIdx = 0; clrIdx < kLEDBuffLength; clrIdx++) {
    m_ledBuffer[clrIdx].SetRGB(0, 0, 0);
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

void RobotContainer::SnakeBOI() {
  ClearLED();
  if (m_previousSnakeIndex == kLEDBuffLength - 1) {
    m_previousSnakeIndex = 0;
  } else {
    m_previousSnakeIndex++;
  }
  m_ledBuffer[m_previousSnakeIndex].SetRGB(255, 0, 0);
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
  // If auto switch 7 is selected, activate gaming mode
  // if (m_currentSelectedAuto == SelectedAuto::kWeBeGaming) {
  //   int firstPixelHue = 0.0;
  //   // For every pixel
  //   for (int i = 0; i < kLEDBuffLength; i++) {
  //     // Calculate the hue - hue is easier for rainbows because the color
  //     // shape is a circle so only one value needs to precess
  //     const auto pixelHue = (firstPixelHue + (i * 180 / kLEDBuffLength)) %
  //     180;
  //     // Set the value
  //     m_ledBuffer[i].SetHSV(pixelHue, 255, 128);

  //     // Increase by to make the rainbow "move"
  //     firstPixelHue += 3;
  //     // Check bounds
  //     firstPixelHue %= 180;
  //   }
  //   m_leds.SetData(m_ledBuffer);
  //   return;
  // }

  // ClearLED();

  auto selectedAutoID = static_cast<size_t>(m_currentSelectedAuto);
  std::array<std::array<std::tuple<int, int, int>, kLEDStripLength>, 4>
      stripBuffers;
  // show which auto is selected
  for (size_t selectedAutoIdx = 0; selectedAutoIdx < selectedAutoID;
       selectedAutoIdx++) {
    for (size_t chunkIdx = 0; chunkIdx < 2; chunkIdx++) {
      for (size_t stripIdx = 0; stripIdx < 4; stripIdx++) {
        stripBuffers[stripIdx][selectedAutoIdx * 3 + chunkIdx] = {
            m_isBlue ? 0 : 255, 0, m_isBlue ? 255 : 0};
      }
    }
  }
  // check if robot is staged in correct location
  auto currentPose = m_drive.GetPose();
  // -2 makes sure lights aren't green if no auto selected
  std::tuple<int, int, int> errors{-2, -2, -2};
  switch (m_currentSelectedAuto) {
    case SelectedAuto::kNorth2ConeChgstat:
      errors = _poseWithin(currentPose,
                           North2ConeChgstat::GetStartingPose(m_isBlue));
      break;
    case SelectedAuto::kSouth2Cone:
      errors = _poseWithin(currentPose, South2Cone::GetStartingPose(m_isBlue));
      break;
    case SelectedAuto::kNorth3Cone:
      errors = _poseWithin(currentPose, North3Cone::GetStartingPose(m_isBlue));
      break;
    case SelectedAuto::kMid1ConeChgstat:
      errors =
          _poseWithin(currentPose, Mid1ConeChgstat::GetStartingPose(m_isBlue));
      break;
    default:
      break;
  }
  if (std::get<0>(errors) == 0 &&
      std::get<1>(errors) == 0) {  // if translation is good
    for (size_t goodPoseIdx = kLEDStripLength - 1;
         goodPoseIdx >= kLEDStripLength - 1 - 4; goodPoseIdx--) {
      for (size_t stripIdx = 0; stripIdx < 4; stripIdx++) {
        stripBuffers[stripIdx][goodPoseIdx] = {0, 255, 0};
      }
    }
  }
  if (std::get<2>(errors) == 0) {  // if rotation is good
    for (size_t goodPoseIdx = kLEDStripLength - 1 - 3;
         goodPoseIdx >= kLEDStripLength - 1 - 3 - 2; goodPoseIdx--) {
      for (size_t stripIdx = 0; stripIdx < 4; stripIdx++) {
        stripBuffers[stripIdx][goodPoseIdx] = {0, 255, 0};
      }
    }
  }
  if (std::get<0>(errors) == +1) {  // if too far right
    for (size_t badPoseIdx = kLEDStripLength - 1;
         badPoseIdx >= kLEDStripLength - 1 - 2; badPoseIdx--) {
      stripBuffers[1][badPoseIdx] = {255, 0, 0};
      stripBuffers[2][badPoseIdx] = {255, 0, 0};
    }
  }
  if (std::get<0>(errors) == -1) {  // if too far left
    for (size_t badPoseIdx = kLEDStripLength - 1;
         badPoseIdx >= kLEDStripLength - 1 - 2; badPoseIdx--) {
      stripBuffers[0][badPoseIdx] = {255, 0, 0};
      stripBuffers[3][badPoseIdx] = {255, 0, 0};
    }
  }
  if (std::get<1>(errors) == +1) {  // if too far up
    for (size_t badPoseIdx = kLEDStripLength - 1;
         badPoseIdx >= kLEDStripLength - 1 - 2; badPoseIdx--) {
      stripBuffers[2][badPoseIdx] = {255, 0, 0};
      stripBuffers[3][badPoseIdx] = {255, 0, 0};
    }
  }
  if (std::get<1>(errors) == -1) {  // if too far down
    for (size_t badPoseIdx = kLEDStripLength - 1;
         badPoseIdx >= kLEDStripLength - 1 - 2; badPoseIdx--) {
      stripBuffers[0][badPoseIdx] = {255, 0, 0};
      stripBuffers[1][badPoseIdx] = {255, 0, 0};
    }
  }
  if (std::get<2>(errors) == +1) {  // if too far counterclockwise
    for (size_t badPoseIdx = kLEDStripLength - 1 - 3;
         badPoseIdx >= kLEDStripLength - 1 - 3 - 2; badPoseIdx--) {
      stripBuffers[0][badPoseIdx] = {255, 0, 0};
      stripBuffers[1][badPoseIdx] = {255, 0, 0};
      stripBuffers[2][badPoseIdx] = {255, 0, 0};
      stripBuffers[3][badPoseIdx] = {255, 0, 0};
    }
  }
  if (std::get<2>(errors) == -1) {  // if too far clockwise
    for (size_t badPoseIdx = kLEDStripLength - 1 - 3;
         badPoseIdx >= kLEDStripLength - 1 - 3 - 2; badPoseIdx--) {
      stripBuffers[0][badPoseIdx] = {0, 0, 255};
      stripBuffers[1][badPoseIdx] = {0, 0, 255};
      stripBuffers[2][badPoseIdx] = {0, 0, 255};
      stripBuffers[3][badPoseIdx] = {0, 0, 255};
    }
  }
  // apply to actual LED strips
  for (size_t stripIdx = 0; stripIdx < 4; stripIdx++) {
    ApplyLEDSingleStrip(stripBuffers[stripIdx], stripIdx);
  }
}
