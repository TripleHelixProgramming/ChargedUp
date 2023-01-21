// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandHelper.h>

#include "subsystems/Gripper.h"

class IntakeCone
    : public frc2::CommandHelper<frc2::CommandBase, IntakeCone> {
 public:
  IntakeCone(Gripper* drive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Gripper* m_gripper;
};
