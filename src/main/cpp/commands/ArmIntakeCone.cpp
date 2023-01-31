// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Superstructure.h"
#include "commands/ArmIntakeCone.h"

using namespace frc;
using namespace units;

ArmIntakeCone::ArmIntakeCone(Superstructure* superstructure)
    : m_superstructure{superstructure} {
  AddRequirements(superstructure);
}

void ArmIntakeCone::Initialize() {}

void ArmIntakeCone::Execute() {
  m_superstructure->ArmIntakeCone();
}

void ArmIntakeCone::End(bool interrupted) {
  m_superstructure->Retract();
}

bool ArmIntakeCone::IsFinished() {
  return m_superstructure->BeamBreakTriggered();
}
