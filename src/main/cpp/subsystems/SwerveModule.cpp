// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/SwerveModule.hpp"

#include <cmath>
#include <numbers>
#include <string>

#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>
#include <units/time.h>
#include <units/current.h>
#include <wpi/DataLog.h>

#include "Constants.hpp"
#include "util/log/TelemetryEntry.hpp"

using namespace frc;
using namespace rev;
using enum rev::CANSparkMaxLowLevel::MotorType;
using namespace ModuleConstants;
using namespace ctre::phoenix::sensors;
using namespace units;
using std::numbers::pi;
using namespace ElectricalConstants;

std::string _GetModuleName(int driveMotorID) {
  switch (driveMotorID) {
    case kDriveMotorPorts[0]:
      return "Front Left";
    case kDriveMotorPorts[1]:
      return "Front Right";
    case kDriveMotorPorts[2]:
      return "Rear Left";
    case kDriveMotorPorts[3]:
      return "Rear Right";
    default:
      return std::to_string(driveMotorID);
  }
}

SwerveModule::SwerveModule(int driveMotorID, int steerMotorID, int absEncoderID)
    : m_driveMotor(driveMotorID, kBrushless),
      m_steerMotor(steerMotorID, kBrushless),
      m_driveEncoder(m_driveMotor.GetEncoder()),
      m_steerEncoder(m_steerMotor.GetEncoder()),
      m_absEncoder(absEncoderID),
      m_driveController(m_driveMotor.GetPIDController()),
      m_steerController(m_steerMotor.GetPIDController()),
      m_name(_GetModuleName(driveMotorID)),
      m_drivePositionLog("Drive/Modules/" + m_name + "/Drive Position (m)"),
      m_driveVelocityLog("Drive/Modules/" + m_name + "/Drive Velocity (mps)"),
      m_steerPositionLog("Drive/Modules/" + m_name + "/Steer Angle (rad)"),
      m_driveVelocitySetpointLog("Drive/Modules/" + m_name +
                                 "/Drive Velocity Setpoint (mps)"),
      m_steerPositionSetpointLog("Drive/Modules/" + m_name +
                                 "/Steer Angle Setpoint (rad)"),
      m_driveSim("SPARK MAX ", driveMotorID),
      m_steerSim("SPARK MAX ", steerMotorID),
      m_driveSimVelocity(m_driveSim.GetDouble("Velocity")),
      m_driveSimPosition(m_driveSim.GetDouble("Position")),
      m_steerSimPosition(m_steerSim.GetDouble("Position")) {
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
  m_steerMotor.EnableVoltageCompensation(12.0);

  m_driveMotor.SetSmartCurrentLimit(80);
  m_steerMotor.SetSmartCurrentLimit(80);

  if constexpr (RobotBase::IsSimulation()) {
    m_simTimer.Start();
  }
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

  // Since we use relative encoder of steer motor, it is a field (doesn't wrap
  // from 2pi to 0 for example). We need to calculate delta to avoid taking a
  // longer route This is analagous to the EnableContinuousInput() function of
  // WPILib's PIDController classes
  double delta = std::fmod(std::fmod((state.angle.Radians().value() -
                                      curAngle.Radians().value() + pi),
                                     2 * pi) +
                               2 * pi,
                           2 * pi) -
                 pi;  // NOLINT

  double adjustedAngle = delta + curAngle.Radians().value();

  m_driveVelocitySetpointLog.Append(state.speed.value());
  m_steerPositionSetpointLog.Append(adjustedAngle);

  m_steerController.SetReference(adjustedAngle,
                                 CANSparkMax::ControlType::kPosition);
  m_driveController.SetReference(state.speed.value(),
                                 CANSparkMax::ControlType::kVelocity);

  if constexpr (RobotBase::IsSimulation()) {
    m_steerSimPosition.Set(adjustedAngle);
  }
}

void SwerveModule::Periodic() {
  m_drivePositionLog.Append(m_driveEncoder.GetPosition());
  m_driveVelocityLog.Append(m_driveEncoder.GetVelocity());
  m_steerPositionLog.Append(m_steerEncoder.GetPosition());
}

void SwerveModule::SimulationPeriodic() {
  units::second_t dt = m_simTimer.Get();
  m_simTimer.Reset();
  m_driveSimPosition.Set(m_driveSimPosition.Get() +
                         m_driveSimVelocity.Get() * dt.value());
}

void SwerveModule::ResetEncoders() {
  m_steerEncoder.SetPosition(0.0);

  CANCoderConfiguration curConfig;
  m_absEncoder.GetAllConfigs(curConfig);
  m_absEncoder.ConfigMagnetOffset(curConfig.magnetOffsetDegrees -
                                  m_absEncoder.GetAbsolutePosition());
}
