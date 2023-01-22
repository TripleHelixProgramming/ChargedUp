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

#include <frc/RobotBase.h>

#include "Constants.h"
#include "units/angle.h"
#include "units/velocity.h"
#include "util/SimSwerveModule.h"

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
      id{driveMotorID},
      m_sim(std::make_unique<SimSwerveModule>(id)) {
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
  if (RobotBase::IsSimulation()) {
    return m_sim->GetState();
  }
  return {meters_per_second_t{m_driveEncoder.GetVelocity()},
          radian_t{m_steerEncoder.GetPosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  if (RobotBase::IsSimulation()) {
    return m_sim->GetPosition();
  }
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

  SmartDashboard::PutNumber("Modules/" + std::to_string(id) + "/Target Velocity",
                            state.speed.value());
  SmartDashboard::PutNumber("Modules/" + std::to_string(id) + "/Actual Velocity",
                            m_driveEncoder.GetVelocity());

  SmartDashboard::PutNumber("Modules/" + std::to_string(id) + "/Target Angle",
                            adjustedAngle);
  SmartDashboard::PutNumber("Modules/" + std::to_string(id) + "/Actual Angle",
                            m_steerEncoder.GetPosition());

  m_steerController.SetReference(adjustedAngle,
                                 CANSparkMax::ControlType::kPosition);
  m_driveController.SetReference(state.speed.value(),
                                 CANSparkMax::ControlType::kVelocity);

  SwerveModuleState adjustedState = {state.speed, units::radian_t{adjustedAngle}};

  m_sim->SetDesiredState(adjustedState);
}

// This method will be called once per scheduler run
void SwerveModule::Periodic() {}

void SwerveModule::ResetEncoders() {
  m_steerEncoder.SetPosition(0.0);

  m_absEncoder.SetPosition(0.0);

  CANCoderConfiguration curConfig;
  m_absEncoder.GetAllConfigs(curConfig);
  m_absEncoder.ConfigMagnetOffset(curConfig.magnetOffsetDegrees -
                                  m_absEncoder.GetAbsolutePosition());
}
