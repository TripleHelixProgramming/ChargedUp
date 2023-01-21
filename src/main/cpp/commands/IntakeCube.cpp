// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Gripper.h"
#include "commands/IntakeCube.h"

using namespace frc;
using namespace units;

IntakeCube::IntakeCube(Gripper* gripper)
    : m_gripper{gripper} {
  AddRequirements(gripper);
}

void IntakeCube::Initialize() {}

void IntakeCube::Execute() {
  m_gripper->IntakeCube();
}

void IntakeCube::End(bool interrupted) {
  m_gripper->Retract();
}

bool IntakeCube::IsFinished() {
  return m_gripper->BeamBreakTriggered();
}
