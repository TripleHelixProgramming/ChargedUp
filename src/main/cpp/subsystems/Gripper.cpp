// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Gripper.h"

#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMaxLimitSwitch.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace rev;

Gripper::Gripper() : m_beamBreak{m_leftWheel.GetForwardLimitSwitch(SparkMaxLimitSwitch::Type::kNormallyOpen)} {
  m_leftWheel.SetSmartCurrentLimit(5);
  m_rightWheel.SetSmartCurrentLimit(5);

  m_leftWheel.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  m_rightWheel.SetIdleMode(CANSparkMax::IdleMode::kBrake);
}

void Gripper::IntakeCube() {
  m_leftWheel.Set(0.5);
  m_rightWheel.Set(-0.5);
  m_lifter.Set(DoubleSolenoid::kForward);
  m_expander.Set(DoubleSolenoid::kReverse);
}

void Gripper::IntakeCone() {
  m_leftWheel.Set(0.5);
  m_rightWheel.Set(-0.5);
  m_lifter.Set(DoubleSolenoid::kForward);
  m_expander.Set(DoubleSolenoid::kForward);
}

void Gripper::EjectGamePiece() {
  m_leftWheel.Set(-0.5);
  m_rightWheel.Set(0.5);
}

void Gripper::Retract() {
  m_leftWheel.Set(0);
  m_rightWheel.Set(0);
  m_lifter.Set(DoubleSolenoid::kReverse);
}

bool Gripper::BeamBreakTriggered() {
  bool data = m_beamBreak.Get();
  SmartDashboard::PutBoolean("Limit switch", data);
  return data;
}

void Gripper::Periodic() {
  BeamBreakTriggered();
}