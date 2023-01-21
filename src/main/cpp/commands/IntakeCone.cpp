// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Gripper.h"
#include "commands/IntakeCone.h"

using namespace frc;
using namespace units;

IntakeCone::IntakeCone(Gripper* gripper)
    : m_gripper{gripper} {
  AddRequirements(gripper);
}

void IntakeCone::Initialize() {}

void IntakeCone::Execute() {
  m_gripper->IntakeCone();
}

void IntakeCone::End(bool interrupted) {
  m_gripper->Retract();
}

bool IntakeCone::IsFinished() {
  return m_gripper->BeamBreakTriggered();
}
