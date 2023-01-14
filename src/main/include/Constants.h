#pragma once

#include <array>
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
  // ----------------------------------- {rear-right, front-right, front-left, rear-left};
  constexpr std::array driveMotorPorts = {        10,          12,         22,        24};
  constexpr std::array steerMotorPorts = {        11,          13,         23,        25};
  constexpr std::array absEncoderPorts = {        31,          33,         43,        45};

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
} // namespace ModuleConstants