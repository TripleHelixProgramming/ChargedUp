// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.hpp"

class ResetAbsoluteEncoders
    : public frc2::CommandHelper<frc2::CommandBase, ResetAbsoluteEncoders> {
 public:
  explicit ResetAbsoluteEncoders(SwerveDrive* drive);

  void Initialize() override;

  bool RunsWhenDisabled() const override;

  bool IsFinished() override;

 private:
  SwerveDrive* drive;
};
