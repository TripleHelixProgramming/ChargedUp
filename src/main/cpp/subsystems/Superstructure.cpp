// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Superstructure.h"

#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "Constants.h"
#include "rev/SparkMaxLimitSwitch.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>

using namespace frc;
using namespace rev;
using namespace units;

using namespace SuperstructureConstants;

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

  m_arm.SetSmartCurrentLimit(20);
  m_armFollower.SetSmartCurrentLimit(20);

  m_arm.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  m_armFollower.SetIdleMode(CANSparkMax::IdleMode::kBrake);
}

void Superstructure::SetIntakeWheelSpeed(double speed) {
  m_intakeWheelSpeed = speed;
}

void Superstructure::SetArmPosition(radian_t position) {
  m_armPosition = position;
}

radian_t Superstructure::GetArmPosition() {
  return radian_t{};
}

bool Superstructure::HasGamePiece() {
  return m_beamBreak.Get();
}

void Superstructure::SuperstructurePeriodic() {
  double intakeWheelSpeed = m_intakeWheelSpeed;
  radian_t armPosition = m_armPosition;

  // If we have game piece, don't spin wheels and lift intake off the ground.
  if (HasGamePiece()) {
    intakeWheelSpeed = std::min(intakeWheelSpeed, 0.0);
    armPosition = radian_t{std::max(armPosition.value(), kMinArmPickupPosition.value())};
  }

  // Ensure arm position bounds are not violated.
  armPosition = radian_t{std::min(kMaxArmPosition.value(), std::max(kMinArmPosition.value(), armPosition.value()))};

  // Set intake wheels and pnuematic expander.
  m_leftWheel.Set(intakeWheelSpeed);
  m_leftWheel.Set(-intakeWheelSpeed);
  m_expander.Set(m_expanded ? DoubleSolenoid::kReverse : DoubleSolenoid::kForward);
}