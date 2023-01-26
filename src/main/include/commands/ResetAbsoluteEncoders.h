#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>

#include "subsystems/SwerveDrive.h"

class ResetAbsoluteEncoders : public frc2::CommandHelper<frc2::CommandBase, ResetAbsoluteEncoders> {
 public:
  ResetAbsoluteEncoders(SwerveDrive* drive);

  void Initialize() override;

  bool RunsWhenDisabled() const override;

  bool IsFinished() override;

 private:
  SwerveDrive* drive;
};