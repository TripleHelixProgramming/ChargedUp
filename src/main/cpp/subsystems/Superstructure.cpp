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

  m_relativeEncoder.SetDistancePerPulse(1.0 / 2048.0);

  double pos = GetAbsoluteArmPosition().value();

  SmartDashboard::PutNumber("Initial pos", pos);

  SmartDashboard::PutNumber("pot", 0.0);
}

void Superstructure::SyncEncoders() {
  // m_seed = GetAbsoluteStringPosition() - m_stringEncoder.GetDistance();
  m_seed = GetAbsoluteArmPosition().value() - RawPosition().value();
}

double Superstructure::GetRelativePosition() {
  return RawPosition().value() + m_seed;
}

degree_t Superstructure::RawPosition() {
  return degree_t{-360 * m_relativeEncoder.GetDistance() *
                  kArmEncoderGearRatio};
}

double Superstructure::RawString() {
  return RawPosition().value() + m_seed;
}

void Superstructure::PositionHigh() {
  // Cone angle is higher than cube placing angle
  SetArmPosition(m_expanded ? 35_deg : 30_deg);
}

void Superstructure::PositionMedium() {
  // Cone angle is higher than cube placing angle
  SetArmPosition(m_expanded ? 28.0_deg : 25.0_deg);
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

void Superstructure::IntakeCubeStation() {
  SetArmPosition(28.0_deg);
  SetIntakeWheelSpeed(0.5);
  SetExtenderPosition(false);
}

void Superstructure::IntakeConeStation() {
  SetArmPosition(28.0_deg);
  SetIntakeWheelSpeed(0.5);
  SetExtenderPosition(true);
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

bool Superstructure::HasGamePiece() {
  return m_beamBreak.Get();
}

double Superstructure::GetAbsoluteStringPosition() {
  double position = GetRelativePosition();

  if (position <= m_encoderPositions[0]) {
    return m_stringPositions[0];
  }

  if (position >= m_encoderPositions[m_encoderPositions.size() - 1]) {
    return m_stringPositions[m_stringPositions.size() - 1];
  }

  for (size_t index = 0; index < m_encoderPositions.size(); ++index) {
    if (m_encoderPositions[index] > position) {
      double t = (position - m_encoderPositions[index - 1]) /
                 (m_encoderPositions[index] - m_encoderPositions[index - 1]);
      return (m_stringPositions[index] - m_stringPositions[index - 1]) * t +
             m_stringPositions[index - 1];
    }
  }

  return 0.0;
}

units::degree_t Superstructure::GetStringAngle() {
  double position = GetAbsoluteArmPosition().value();

  if (position <= m_stringPositions[0]) {
    return degree_t{m_encoderPositions[0]};
  }

  if (position >= m_stringPositions[m_stringPositions.size() - 1]) {
    return degree_t{m_encoderPositions[m_encoderPositions.size() - 1]};
  }

  for (size_t index = 0; index < m_stringPositions.size(); ++index) {
    if (m_stringPositions[index] > position) {
      double t = (position - m_stringPositions[index - 1]) /
                 (m_stringPositions[index] - m_stringPositions[index - 1]);
      return degree_t{
          (m_encoderPositions[index] - m_encoderPositions[index - 1]) * t +
          m_encoderPositions[index - 1]};
    }
  }

  return 0.0_deg;
}

void Superstructure::SuperstructurePeriodic() {
  double intakeWheelSpeed = m_intakeWheelSpeed;
  degree_t armPosition = m_armPosition;
  double currentAngle = GetRelativePosition();

  if (m_flipConeMode) {
    armPosition = m_flipConeUp ? 11_deg : 6_deg;
  }

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
  SmartDashboard::PutNumber("Error", armPosition.value() - currentAngle);

  // Set state of hardware.
  m_leftWheel.Set(intakeWheelSpeed);
  m_rightWheel.Set(-intakeWheelSpeed);
  if (m_expanded) {
    m_expander.Set(DoubleSolenoid::kForward);
  } else {
    m_expander.Set(DoubleSolenoid::kReverse);
  }

  m_integral += armPosition.value() - currentAngle;

  double minVoltage = -1.0;
  double maxVoltage = 1.0;
  if (m_kI != 0.0) {
    m_integral =
        std::max(minVoltage / m_kI, std::min(m_integral, maxVoltage / m_kI));
  }

  if (std::abs(armPosition.value() - currentAngle) > m_tolerance) {
    m_integral = 0.0;
  }

  if (armPosition.value() < 10.0) {
    m_integral = 0.0;
  }

  m_armController.SetGoal(armPosition);
  auto commandedVoltage = volt_t{
      m_armController.Calculate(degree_t{currentAngle}) + m_integral * m_kI};

  if (currentAngle < 3.0 && armPosition.value() < 5.0) {
    commandedVoltage = volt_t{-0.6};
  }

  SmartDashboard::PutNumber("Target angle", armPosition.value());

  commandedVoltage = volt_t{std::min(commandedVoltage.value(), 3.0)};

  commandedVoltage =
      volt_t{std::max(std::pow(((30 - currentAngle) / 30.0), 2) * -2 - 1,
                      commandedVoltage.value())};

  SmartDashboard::PutNumber("Applied voltage", commandedVoltage.value());

  m_armLeader.SetVoltage(commandedVoltage);
}
