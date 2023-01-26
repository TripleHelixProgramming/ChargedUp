// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/SwerveModule.h"

#include <cmath>
#include <numbers>

#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>

#include "Constants.h"

using namespace frc;
using namespace rev;
using enum rev::CANSparkMaxLowLevel::MotorType;
using namespace ModuleConstants;
using namespace ctre::phoenix::sensors;
using namespace units;
using std::numbers::pi;

SwerveModule::SwerveModule(int driveMotorID, int steerMotorID, int absEncoderID)
    : m_driveMotor(driveMotorID, kBrushless),
      m_steerMotor(steerMotorID, kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder()),
      m_steerEncoder(m_steerMotor.GetEncoder()),
      m_absEncoder(absEncoderID),
      m_driveController(m_driveMotor.GetPIDController()),
      m_steerController(m_steerMotor.GetPIDController()),
      id{driveMotorID} {
  m_driveController.SetP(kDriveP);
  m_driveController.SetI(kDriveI);
  m_driveController.SetD(kDriveD);
  m_driveController.SetFF(kDriveFF);

  m_steerController.SetP(kSteerP);
  m_steerController.SetI(kSteerI);
  m_steerController.SetD(kSteerD);
  m_steerController.SetFF(kSteerFF);

  m_driveMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
  m_steerMotor.SetIdleMode(CANSparkMax::IdleMode::kBrake);

  m_driveEncoder.SetPositionConversionFactor(
      (2 * pi * kWheelRadius / kDriveGearRatio).value());
  m_driveEncoder.SetVelocityConversionFactor(
      (2 * pi * kWheelRadius / kDriveGearRatio / 60_s).value());
  m_steerEncoder.SetPositionConversionFactor(2 * pi / kSteerGearRatio);

  m_steerEncoder.SetPosition(m_absEncoder.GetAbsolutePosition() * pi /
                             180);  // convert to rad

  m_driveMotor.EnableVoltageCompensation(12.0);
  m_driveMotor.EnableVoltageCompensation(12.0);
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {meters_per_second_t{m_driveEncoder.GetVelocity()},
          radian_t{m_steerEncoder.GetPosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {meter_t{m_driveEncoder.GetPosition()},
          radian_t{m_steerEncoder.GetPosition()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, radian_t{m_steerEncoder.GetPosition()});

  Rotation2d curAngle = radian_t{m_steerEncoder.GetPosition()};

  double delta = std::fmod(std::fmod((state.angle.Radians().value() -
                                      curAngle.Radians().value() + pi),
                                     2 * pi) +
                               2 * pi,
                           2 * pi) -
                 pi;

  double adjustedAngle = delta + curAngle.Radians().value();

  SmartDashboard::PutNumber("Target velocity " + std::to_string(id) + ": ",
                            state.speed.value());

  SmartDashboard::PutNumber("Target angle " + std::to_string(id) + ": ",
                            adjustedAngle);
  

  m_steerController.SetReference(adjustedAngle,
                                 CANSparkMax::ControlType::kPosition);
  m_driveController.SetReference(state.speed.value(),
                                 CANSparkMax::ControlType::kVelocity);
}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {
  SmartDashboard::PutNumber("Actual angle " + std::to_string(id) + ": ",
                            m_steerEncoder.GetPosition());
  SmartDashboard::PutNumber("Actual velocity " + std::to_string(id) + ": ",
                            m_driveEncoder.GetVelocity());
}

void SwerveModule::ResetEncoders() {
  SmartDashboard::PutBoolean("did we do a dumb", false);

  m_steerEncoder.SetPosition(0.0);

  CANCoderConfiguration curConfig;
  m_absEncoder.GetAllConfigs(curConfig);
  m_absEncoder.ConfigMagnetOffset(curConfig.magnetOffsetDegrees -
                                  m_absEncoder.GetAbsolutePosition());
}
