// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Superstructure.hpp"

#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxLimitSwitch.h>
#include <units/angle.h>
#include <units/energy.h>
#include <units/math.h>

#include "Constants.hpp"

using namespace frc;
using namespace rev;
using namespace units;

using namespace SuperstructureConstants;

Superstructure::Superstructure()
    : m_beamBreak{m_leftWheel.GetForwardLimitSwitch(
          SparkMaxLimitSwitch::Type::kNormallyOpen)} {
  // Initialize intake wheel motors
  m_leftWheel.SetSmartCurrentLimit(5);
  m_rightWheel.SetSmartCurrentLimit(5);

  m_leftWheel.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  m_rightWheel.SetIdleMode(CANSparkMax::IdleMode::kBrake);

  // Initialize arm motors
  m_armLeader.EnableVoltageCompensation(12);
  m_armFollower.EnableVoltageCompensation(12);
  m_armLeader.SetSmartCurrentLimit(60);
  m_armFollower.SetSmartCurrentLimit(60);
  m_armLeader.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  m_armFollower.SetIdleMode(CANSparkMax::IdleMode::kBrake);

  m_armFollower.Follow(m_armLeader, true);

  // Initialize arm encoder
  m_armEncoder.SetPositionOffset(kArmEncoderOffset);

  m_armRelativeEncoder.SetDistancePerPulse(1. / 2048.);

  double pos = GetAbsoluteArmPosition().value();

  SmartDashboard::PutNumber("Initial pos", pos);

  SmartDashboard::PutNumber("pot", 0.0);
}

void Superstructure::SyncEncoders() {
  m_armOffset = GetAbsoluteArmPosition() -
                degree_t{-360 * m_armRelativeEncoder.GetDistance() *
                         kArmEncoderGearRatio};
}

void Superstructure::PositionCubeHigh() {}

void Superstructure::PositionCubeMedium() {}

void Superstructure::PositionConeHigh() {
  SetArmPosition(34.25_deg);
}

void Superstructure::PositionConeMedium() {
  SetArmPosition(27.0_deg);
}

void Superstructure::IntakeCone() {
  SetArmPosition(-7.5_deg);
  SetIntakeWheelSpeed(0.5);
  SetExtenderPosition(true);
}

void Superstructure::IntakeCube() {
  SetArmPosition(-7.5_deg);
  SetIntakeWheelSpeed(0.5);
  SetExtenderPosition(false);
}

void Superstructure::SetIntakeWheelSpeed(double speed) {
  m_intakeWheelSpeed = speed;
}

void Superstructure::SetExtenderPosition(bool expanded) {
  m_expanded = expanded;
}

void Superstructure::Outtake() {
  if (!m_expanded) {
    SetIntakeWheelSpeed(-0.5);
  }
  SetExtenderPosition(false);
}

void Superstructure::SetArmPosition(radian_t position) {
  m_armPosition = position;
}

degree_t Superstructure::GetAbsoluteArmPosition() {
  return degree_t{360 * m_armEncoder.GetAbsolutePosition() *
                      kArmEncoderGearRatio -
                  kArmEncoderOffset};
}

degree_t Superstructure::GetArmPosition() {
  return degree_t{-360 * m_armRelativeEncoder.GetDistance() *
                  kArmEncoderGearRatio} +
         m_armOffset;
}

bool Superstructure::HasGamePiece() {
  return m_beamBreak.Get();
}

void Superstructure::SuperstructurePeriodic() {
  double intakeWheelSpeed = m_intakeWheelSpeed;
  degree_t armPosition = m_armPosition;

  // If we have game piece, don't spin wheels and lift intake off the ground.
  if (HasGamePiece()) {
    intakeWheelSpeed = units::math::min(intakeWheelSpeed, 0.0);
    armPosition =
        degree_t{std::max(armPosition.value(), kMinArmPickupPosition.value())};
  }

  // Ensure arm position bounds are not violated.
  armPosition = units::math::min(
      kMaxArmPosition, units::math::max(kMinArmPosition, armPosition));

  SmartDashboard::PutNumber("Intake wheel speed", intakeWheelSpeed);
  SmartDashboard::PutBoolean("Extended", m_expanded);
  SmartDashboard::PutNumber("Arm current", m_armLeader.GetOutputCurrent());
  SmartDashboard::PutNumber("Error", (armPosition - GetArmPosition()).value());

  // Set state of hardware.
  m_leftWheel.Set(intakeWheelSpeed);
  m_rightWheel.Set(-intakeWheelSpeed);
  if (m_expanded) {
    m_expander.Set(DoubleSolenoid::kForward);
  } else {
    m_expander.Set(DoubleSolenoid::kReverse);
  }

  SmartDashboard::PutNumber("Target", armPosition.value());

  m_integral += (armPosition - GetArmPosition()).value();

  double minVoltage = 0.0;
  double maxVoltage = 2.5;
  if (m_kI != 0.0) {
    m_integral =
        std::max(minVoltage / m_kI, std::min(m_integral, maxVoltage / m_kI));
  }

  if (std::abs((armPosition - GetArmPosition()).value()) > m_tolerance) {
    m_integral = 0.0;
  }

  if (armPosition.value() < 10.0) {
    m_integral = 0.0;
  }

  m_armController.SetGoal(armPosition);
  auto commandedVoltage =
      volt_t{m_armController.Calculate(GetArmPosition()) + m_integral * m_kI};

  if (GetArmPosition().value() < 3.0 && armPosition.value() < 1.0) {
    commandedVoltage = volt_t{-0.5};
  }

  commandedVoltage = volt_t{std::min(commandedVoltage.value(), 3.0)};

  commandedVoltage = volt_t{
      std::max(std::pow(((30 - GetArmPosition().value()) / 30.0), 2) * -2 - 1,
               commandedVoltage.value())};

  SmartDashboard::PutNumber("Applied voltage", commandedVoltage.value());

  m_armLeader.SetVoltage(commandedVoltage);
}
