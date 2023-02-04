// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <string>

#include <ctre/phoenix/sensors/CANCoder.h>
#include <frc/Timer.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>

#include "util/log/DoubleTelemetryEntry.hpp"

class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(int driveMotorID, int steerMotorID, int absEncoderID);

  frc::SwerveModuleState GetState() const;

  frc::SwerveModulePosition GetPosition() const;

  void SetDesiredState(const frc::SwerveModuleState& state);

  void Periodic() override;

  void SimulationPeriodic() override;

  void ResetEncoders();

 private:
  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_steerMotor;

  rev::SparkMaxRelativeEncoder m_driveEncoder;
  rev::SparkMaxRelativeEncoder m_steerEncoder;

  ctre::phoenix::sensors::CANCoder m_absEncoder;

  rev::SparkMaxPIDController m_driveController;
  rev::SparkMaxPIDController m_steerController;

  std::string m_name;

  DoubleTelemetryEntry m_drivePositionLog;
  DoubleTelemetryEntry m_driveVelocityLog;
  DoubleTelemetryEntry m_steerPositionLog;
  DoubleTelemetryEntry m_driveVelocitySetpointLog;
  DoubleTelemetryEntry m_steerPositionSetpointLog;

  // Simulation
  frc::Timer m_simTimer;

  frc::sim::SimDeviceSim m_driveSim;
  frc::sim::SimDeviceSim m_steerSim;

  hal::SimDouble m_driveSimVelocity;
  hal::SimDouble m_driveSimPosition;
  hal::SimDouble m_steerSimPosition;
};
