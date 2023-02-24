// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Superstructure.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "util/TrajectoryManager.hpp"

class OneConeChgstat
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, OneConeChgstat> {
 public:
  OneConeChgstat(SwerveDrive* drive, Superstructure* superstructure,
                 const TrajectoryManager* trajManager);

 private:
  SwerveDrive* m_drive;
  const TrajectoryManager* m_trajManager;
};
