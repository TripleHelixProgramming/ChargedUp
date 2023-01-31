// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Superstructure.h"
#include "commands/ArmIntakeCube.h"

using namespace frc;
using namespace units;

ArmIntakeCube::ArmIntakeCube(Superstructure* superstructure)
    : m_superstructure{superstructure} {
  AddRequirements(superstructure);
}

void ArmIntakeCube::Initialize() {}

void ArmIntakeCube::Execute() {
  m_superstructure->ArmIntakeCube();
}

void ArmIntakeCube::End(bool interrupted) {
  m_superstructure->Retract();
}

bool ArmIntakeCube::IsFinished() {
  return m_superstructure->BeamBreakTriggered();
}
