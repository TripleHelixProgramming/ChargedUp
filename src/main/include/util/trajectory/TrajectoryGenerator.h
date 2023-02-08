// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <vector>

#include <frc/geometry/Pose2d.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>

#include "util/trajectory/Trajectory.h"
#include "util/trajectory/TrajectoryConfig.h"
#include "util/trajectory/constraint/TrajectoryConstraint.h"

using namespace units::literals;

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(TrajectoryConfig& config);

  Trajectory Generate(frc::Pose2d start, frc::Pose2d end);

 private:
  TrajectoryConfig m_config;
};
