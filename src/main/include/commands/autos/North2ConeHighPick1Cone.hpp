// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Superstructure.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "util/TrajectoryManager.hpp"

class North2ConeHighPick1Cone
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, North2ConeHighPick1Cone> {
 public:
  North2ConeHighPick1Cone(SwerveDrive* drive, Superstructure* superstructure, bool isBlue);

  static frc::Pose2d GetStartingPose(bool isBlue);
};
