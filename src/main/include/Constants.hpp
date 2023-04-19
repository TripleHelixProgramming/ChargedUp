// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <array>
#include <numbers>

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <opencv2/core/mat.hpp>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include "util/log/TelemetryEntry.hpp"

namespace ElectricalConstants {

constexpr int kRobotControllerPort = 0;
constexpr int kPowerDistributionPort = 1;

constexpr int kPHPort = 1;

// ------------------------------- {fl, fr, rl, rr};
constexpr int kDriveMotorPorts[] = {28, 26, 10, 12};
constexpr int kSteerMotorPorts[] = {29, 27, 11, 13};
constexpr int kAbsEncoderPorts[] = {43, 33, 45, 31};

constexpr int kArmLeaderPort = 14;
constexpr int kArmFollowerPort = 15;

constexpr int kIntakeLeftWheelPort = 16;
constexpr int kIntakeRightWheelPort = 17;

constexpr int kArmEncoderPort = 0;

/// Maps rotary switch positions/indices to digital input pins on the RIO
constexpr std::array<int, 8> kAutoSwitchPorts = {11, 12, 13, 18,
                                                 19, 20, 21, 22};
constexpr int kRedBlueSwitchPort = 10;

/// LED strip
constexpr int kLEDStripLength = 22;
constexpr int kAltLEDStripLength = 18;
constexpr int kLEDBuffLength = kLEDStripLength * 4 + kAltLEDStripLength;
constexpr std::array<bool, 4> kStripDirections = {
    false, true, true, true};  // false means up, true means down
}  // namespace ElectricalConstants

namespace DriveConstants {

constexpr units::meter_t kWheelBase = 0.6223_m;
constexpr units::meter_t kTrackWidth = 0.5715_m;

constexpr auto kMaxVelocityX = 4.0_mps;
constexpr auto kMaxVelocityY = 4.0_mps;
constexpr auto kMaxVelocityAngular = 5.0_rad_per_s;

}  // namespace DriveConstants

namespace ModuleConstants {

constexpr double kDriveP = 0.1;
constexpr double kDriveI = 0.0;
constexpr double kDriveD = 0.0;
constexpr double kDriveFF = 0.225;

constexpr double kSteerP = 1.5;
constexpr double kSteerI = 0.0;
constexpr double kSteerD = 0.0;
constexpr double kSteerFF = 0.0;

constexpr auto kMaxSpeed = 4.0_mps;

constexpr auto kWheelRadius = 0.047804878_m;

constexpr double kDriveGearRatio = 6.75;
constexpr double kSteerGearRatio = 12.8;

}  // namespace ModuleConstants

namespace SuperstructureConstants {

constexpr auto kMinArmPosition = -7.5_deg;
constexpr auto kMinArmPickupPosition = -1.0_deg;
constexpr auto kMaxArmPosition = 1.0_rad;

constexpr double kArmFF = 0;
constexpr double kArmP = 20.0;
constexpr double kArmI = 0;
constexpr double kArmD = 4.0;

constexpr double kArmEncoderOffset = -47.0 + 0.0;
constexpr double kArmEncoderGearRatio = 1 / 5.0;

}  // namespace SuperstructureConstants

namespace VisionConstants {

// NOLINT
const frc::Transform3d kRobotToLeftCam(frc::Translation3d(10.984_in, 12.749_in,
                                                          24.671_in),
                                       frc::Rotation3d(0_deg, 0_deg, -45_deg));

// NOLINT
const frc::Transform3d kRobotToRightCam(frc::Translation3d(10.984_in,
                                                           -12.749_in,
                                                           24.671_in),
                                        frc::Rotation3d(0_deg, 0_deg, 42_deg));

// NOLINT
const frc::Transform3d kRobotToBackCam(frc::Translation3d(-11.760_in, 0.000_in,
                                                          24.671_in),
                                       frc::Rotation3d(0_deg, 0_deg, 180_deg));
}  // namespace VisionConstants

namespace OIConstants {

constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;

// Xbox Joystick Axis
constexpr int kXboxLeftXAxis = 0;
constexpr int kXboxLeftYAxis = 1;
constexpr int kXboxLeftTrigger = 2;
constexpr int kXboxRightTripper = 3;
constexpr int kXboxRightXAxis = 4;
constexpr int kXboxRightYAxis = 5;

// Xbox Button Bindings
constexpr int kXboxA = 1;
constexpr int kXboxB = 2;
constexpr int kXboxX = 3;
constexpr int kXboxY = 4;
constexpr int kXboxLB = 5;
constexpr int kXboxRB = 6;
constexpr int kXboxView = 7;
constexpr int kXboxMenuT = 8;
constexpr int kXboxLeftStickButton = 9;
constexpr int kXboxRightStickButton = 10;

// RadioMaster Zorro Joystick Axis
constexpr int kZorroLeftXAxis = 0;
constexpr int kZorroLeftYAxis = 1;
constexpr int kZorroLeftDial = 2;
constexpr int kZorroRightDial = 3;
constexpr int kZorroRightXAxis = 4;
constexpr int kZorroRightYAxis = 5;

// RadioMaster Zorro Button Bindings
constexpr int kZorroBDown = 1;
constexpr int kZorroBMid = 2;
constexpr int kZorroBUp = 3;
constexpr int kZorroEDown = 4;
constexpr int kZorroEUp = 5;
constexpr int kZorroAIn = 6;
constexpr int kZorroGIn = 7;
constexpr int kZorroCDown = 8;
constexpr int kZorroCMid = 9;
constexpr int kZorroCUp = 10;
constexpr int kZorroFDown = 11;
constexpr int kZorroFUp = 12;
constexpr int kZorroDIn = 13;
constexpr int kZorroHIn = 14;

}  // namespace OIConstants

constexpr TelemetryLevel kTelemetryLevel = TelemetryLevel::kDebug;
