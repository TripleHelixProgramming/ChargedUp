// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Superstructure.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "util/TrajectoryManager.hpp"

class South2ConeHigh
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, South2ConeHigh> {
 public:
  South2ConeHigh(SwerveDrive* drive, Superstructure* superstructure, bool isBlue);

  static frc::Pose2d GetStartingPose(bool isBlue);
};
