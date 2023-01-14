// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <subsystems/SwerveModule.h>
#include <Constants.h>
#include <array>
#include <AHRS.h>
#include <units/angle.h>

class SwerveDrive : public frc2::SubsystemBase {
 public:
  SwerveDrive();

  void Periodic() override;

  frc::Pose2d GetPose() const;

  void ResetOdometry(const frc::Pose2d& pose);

  frc::Rotation2d GetGyroHeading();

  void Drive(frc::ChassisSpeeds speeds);

  void Brake();
 private:
  AHRS m_gyro{frc::SPI::Port::kMXP};

  std::array<SwerveModule, 4> m_modules{
    SwerveModule(driveMotorPorts[0], steerMotorPorts[0], absEncoderPorts[0]),
    SwerveModule(driveMotorPorts[1], steerMotorPorts[1], absEncoderPorts[1]),
    SwerveModule(driveMotorPorts[2], steerMotorPorts[2], absEncoderPorts[2]),
    SwerveModule(driveMotorPorts[3], steerMotorPorts[3], absEncoderPorts[3])};

  frc::SwerveDriveKinematics<4> m_kinematics{
    frc::Translation2d(DriveConstants::kWheelBase / 2.0, DriveConstants::kTrackWidth / 2.0),
    frc::Translation2d(DriveConstants::kWheelBase / 2.0, -DriveConstants::kTrackWidth / 2.0),
    frc::Translation2d(-DriveConstants::kWheelBase / 2.0, DriveConstants::kTrackWidth / 2.0),
    frc::Translation2d(-DriveConstants::kWheelBase / 2.0, -DriveConstants::kTrackWidth / 2.0)};

  // STATE:

  frc::SwerveDriveOdometry<4> m_odometry{
    m_kinematics,
    Rotation2d(units::degree_t{m_gyro.GetYaw()}),
               {m_modules[0].GetPosition(), m_modules[1].GetPosition(), 
               m_modules[2].GetPosition(), m_modules[3].GetPosition()}};

  frc::Rotation2d fieldRelativeAngleOffset;

  void UpdateOdometry();

  void NormalizeDrive(const std::array<frc::SwerveModuleState, 4>& desiredStates,
      const frc::ChassisSpeeds& speeds);
};
