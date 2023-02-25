// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Superstructure.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "util/TrajectoryManager.hpp"

class Mid1ConeChgstat : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                                   Mid1ConeChgstat> {
 public:
  Mid1ConeChgstat(SwerveDrive* drive, Superstructure* superstructure,
                  const TrajectoryManager* trajManager, bool isBlue);
};
