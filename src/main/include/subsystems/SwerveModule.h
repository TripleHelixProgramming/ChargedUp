// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <ctre/phoenix/sensors/CANCoder.h>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>

class SwerveModule : public frc2::SubsystemBase {
 public:
  SwerveModule(int driveMotorID, int steerMotorID, int absEncoderID);

  frc::SwerveModuleState GetState() const;

  frc::SwerveModulePosition GetPosition() const;

  void SetDesiredState(const frc::SwerveModuleState& state);

  void Periodic() override;

  void ResetEncoders();

 private:
  rev::CANSparkMax m_driveMotor;
  rev::CANSparkMax m_steerMotor;

  rev::SparkMaxRelativeEncoder m_driveEncoder;
  rev::SparkMaxRelativeEncoder m_steerEncoder;

  ctre::phoenix::sensors::CANCoder m_absEncoder;

  rev::SparkMaxPIDController m_driveController;
  rev::SparkMaxPIDController m_steerController;

  int id;
};
