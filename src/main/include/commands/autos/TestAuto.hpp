// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Superstructure.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "util/TrajectoryManager.hpp"

class TestAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, TestAuto> {
 public:
  TestAuto(SwerveDrive* drive, Superstructure* superstructure);

  static frc::Pose2d GetStartingPose(bool isBlue);
};
