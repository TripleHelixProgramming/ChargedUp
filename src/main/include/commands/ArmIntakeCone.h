// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandHelper.h>

#include "subsystems/Superstructure.h"

class ArmIntakeCone
    : public frc2::CommandHelper<frc2::CommandBase, ArmIntakeCone> {
 public:
  ArmIntakeCone(Superstructure* drive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Superstructure* m_superstructure;
};
