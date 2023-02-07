// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>

#include "util/trajectory/Trajectory.h"

using namespace units::literals;

class TrajectoryGenerator {
 public:
  Trajectory Generate(frc::Pose2d start, frc::Pose2d end);

 private:
  // Acceleration/velocity constraints
  //
  // TODO: move these into a separate modular constraint class
  units::meters_per_second_squared_t maxAccelerationX = 2.5_mps_sq;
  units::meters_per_second_squared_t maxAccelerationY = 2.5_mps_sq;
  units::radians_per_second_squared_t maxRotationalAcceleration =
      5_rad_per_s_sq;

  units::meters_per_second_t maxVelocityX = 2.5_mps;
  units::meters_per_second_t maxVelocityY = 2.5_mps;
  units::radians_per_second_t maxRotationalVelocity = 5_rad_per_s;
};
