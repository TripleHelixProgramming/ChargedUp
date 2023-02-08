// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <array>
#include <numbers>

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>

#include "util/log/TelemetryEntry.hpp"

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace ElectricalConstants {

constexpr int kRobotControllerPort = 0;
constexpr int kPowerDistributionPort = 1;
// ------------------------------- {fl, fr, rl, rr};
constexpr int kDriveMotorPorts[] = {28, 26, 10, 12};
constexpr int kSteerMotorPorts[] = {29, 27, 11, 13};
constexpr int kAbsEncoderPorts[] = {43, 33, 45, 31};

constexpr int kArmLeaderPort = 14;
constexpr int kArmFollowerPort = 15;

constexpr int kIntakeLeftWheelPort = 16;
constexpr int kIntakeRightWheelPort = 17;

constexpr int kArmEncoderPort = 0;

}  // namespace ElectricalConstants

namespace DriveConstants {

constexpr units::meter_t kWheelBase = 0.6223_m;
constexpr units::meter_t kTrackWidth = 0.5715_m;

constexpr auto kMaxVelocityX = 4.0_mps;
constexpr auto kMaxVelocityY = 4.0_mps;
constexpr auto kMaxVelocityAngular = 4.0_rad_per_s;

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

constexpr auto kWheelRadius = 0.049_m;

constexpr double kDriveGearRatio = 6.75;
constexpr double kSteerGearRatio = 12.8;

}  // namespace ModuleConstants

namespace SuperstructureConstants {

constexpr auto kMinArmPosition = 0.0_deg;
constexpr auto kMinArmPickupPosition = 3.0_deg;
constexpr auto kMaxArmPosition = 1.0_rad;

constexpr double kArmFF = 0;
constexpr double kArmP = 10.0;
constexpr double kArmI = 0;
constexpr double kArmD = 0;

constexpr double kArmEncoderOffset = 17;
constexpr double kArmEncoderGearRatio = 1 / 5.0;

}  // namespace SuperstructureConstants

namespace VisionConstants {

// const frc::Transform3d kRobotToCam(frc::Translation3d(0.0_m,
// -13.5_in, 11.2_in),
//                                    frc::Rotation3d(0_rad, 0_rad, -90_deg));

const frc::Transform3d kRobotToCam(frc::Translation3d(0.0_m, 0.0_m, 0.0_m),
                                   frc::Rotation3d(0_rad, 0_rad, 0.0_rad));

}  // namespace VisionConstants

namespace OIConstants {

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
