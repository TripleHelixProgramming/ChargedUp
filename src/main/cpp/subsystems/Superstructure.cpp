// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Superstructure.h"

#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "rev/SparkMaxLimitSwitch.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace rev;

Superstructure::Superstructure() : m_beamBreak{m_leftWheel.GetForwardLimitSwitch(SparkMaxLimitSwitch::Type::kNormallyOpen)} {
  // Initialize intake wheel motors
  m_leftWheel.SetSmartCurrentLimit(5);
  m_rightWheel.SetSmartCurrentLimit(5);

  m_leftWheel.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  m_rightWheel.SetIdleMode(CANSparkMax::IdleMode::kBrake);

  // Initialize arm motors
  m_arm.EnableVoltageCompensation(12);
  m_armFollower.EnableVoltageCompensation(12);

  m_armFollower.Follow(m_arm, true);

  m_arm.SetSmartCurrentLimit(60);
  m_armFollower.SetSmartCurrentLimit(60);

  m_arm.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  m_armFollower.SetIdleMode(CANSparkMax::IdleMode::kBrake);
}

void Superstructure::IntakeCube() {
  m_leftWheel.Set(0.5);
  m_rightWheel.Set(-0.5);
  m_expander.Set(DoubleSolenoid::kReverse);
}

void Superstructure::IntakeCone() {
  m_leftWheel.Set(0.5);
  m_rightWheel.Set(-0.5);
  m_expander.Set(DoubleSolenoid::kForward);
}

void Superstructure::EjectGamePiece() {
  m_leftWheel.Set(-0.5);
  m_rightWheel.Set(0.5);
}

void Superstructure::Retract() {
  m_leftWheel.Set(0);
  m_rightWheel.Set(0);
  m_lifter.Set(DoubleSolenoid::kReverse);
}

bool Superstructure::BeamBreakTriggered() {
  bool data = m_beamBreak.Get();
  SmartDashboard::PutBoolean("Limit switch", data);
  return data;
}

void Superstructure::Periodic() {
  BeamBreakTriggered();
}