// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/ResetAbsoluteEncoders.h"

#include "subsystems/SwerveDrive.h"

ResetAbsoluteEncoders::ResetAbsoluteEncoders(SwerveDrive* drive)
    : drive(drive) {}

void ResetAbsoluteEncoders::Initialize() {
  drive->ResetAbsoluteEncoders();
}

bool ResetAbsoluteEncoders::RunsWhenDisabled() const {
  return true;
}

bool ResetAbsoluteEncoders::IsFinished() {
  return true;
}
