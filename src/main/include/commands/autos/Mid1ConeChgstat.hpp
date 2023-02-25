// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <frc/geometry/Pose2d.h>

#include "subsystems/Superstructure.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "util/TrajectoryManager.hpp"

class Mid1ConeChgstat : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                                   Mid1ConeChgstat> {
 public:
  Mid1ConeChgstat(SwerveDrive* drive, Superstructure* superstructure, bool isBlue);

  static frc::Pose2d GetStartingPose(bool isBlue);
};
