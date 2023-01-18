// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Gripper.h"

#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>

using namespace frc;
using namespace rev;

Gripper::Gripper() {
  m_leftWheel.SetSmartCurrentLimit(5);
  m_rightWheel.SetSmartCurrentLimit(5);

  m_leftWheel.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  m_rightWheel.SetIdleMode(CANSparkMax::IdleMode::kBrake);
}

void Gripper::SetWheelSpeeds(double power) {
  m_leftWheel.Set(power);
  m_rightWheel.Set(-power);
}

void Gripper::Extend() {
  m_expander.Set(DoubleSolenoid::kForward);
}

void Gripper::Retract() {
  m_expander.Set(DoubleSolenoid::kReverse);
}
