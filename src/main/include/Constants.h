// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>

#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/angular_acceleration.h>

namespace OperatorConstants {

  constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace ElectricalConstants {

  constexpr int kRobotControllerPort = 0;
  constexpr int kPowerDistributionPort = 1;

  constexpr int kRearRightDriveMotorPort = 10;
  constexpr int kFrontRightDriveMotorPort = 12;
  constexpr int kFrontLeftDriveMotorPort = 22;
  constexpr int kRearLeftDriveMotorPort = 24;

  constexpr int kRearRightSteerMotorPort = 11;  
  constexpr int kFrontRightSteerMotorPort = 13;
  constexpr int kFrontLeftSteerMotorPort = 23;
  constexpr int kRearLeftSteerMotorPort = 25;

  constexpr int kRearRightSteerEncoderPort = 31;
  constexpr int kFrontRightSteerEncoderPort = 33;
  constexpr int kFrontLeftSteerEncoderPort = 43;
  constexpr int kRearLeftSteerEncoderPort = 45;

} // namespace ElectricalConstants

namespace DriveConstants {

  constexpr units::meter_t kWheelBase = 0.6223_m;
  constexpr units::meter_t kTrackWidth = 0.5715_m;

} // namespace DriveConstants

namespace ModuleConstants {

  constexpr double kDriveP = 0.1;
  constexpr double kDriveI = 0.0;
  constexpr double kDriveD = 0.0;
  constexpr double kDriveFF = 2.96;

  constexpr double kSteerP = 0.01;
  constexpr double kSteerI = 0.0;
  constexpr double kSteerD = 0.005;
  constexpr double kSteerFF = 0.0;

  constexpr auto kMaxSpeed = 4.0_mps;

  constexpr auto kWheelRadius = 0.049_m;

  constexpr double kDriveGearRatio = 6.75;
  constexpr double kSteerGearRatio = 12.8;
}