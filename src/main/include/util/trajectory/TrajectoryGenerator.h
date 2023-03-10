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

namespace trajectory {

Trajectory Generate(frc::Pose2d start, frc::Pose2d end, TrajectoryConfig& config);

}  // namespace trajectory
