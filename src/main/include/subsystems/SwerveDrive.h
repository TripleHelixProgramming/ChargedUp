// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <subsystems/SwerveModule.h>
#include <Constants.h>
#include <array>
#include <AHRS.h>

class SwerveDrive : public frc2::SubsystemBase {
 public:
  SwerveDrive();  

  void Periodic() override;

  frc::Pose2d GetPose();

  void ResetOdometry(frc::Pose2d pose);

  void Drive(frc::ChassisSpeeds speeds);

  void Brake();
 private:
  AHRS m_gyro;

  SwerveModule m_frontLeftModule(ElectricalConstants::kFrontLeftDriveMotorPort,
                                 ElectricalConstants::kFrontLeftTurningMotorPort,
                                 ElectricalConstants::kFrontLeftTurningEncoderPort,
                                 ModuleConstants::k);
  SwerveModule m_frontRightModule{};
  SwerveModule m_backLeftModule{};
  SwerveModule m_backRightModule{};

  frc::SwerveDriveKinematics<4> m_kinematics{
    frc::Translation2d(DriveConstants::kWheelBase / 2.0, DriveConstants::kTrackWidth / 2.0),
    frc::Translation2d(DriveConstants::kWheelBase / 2.0, -DriveConstants::kTrackWidth / 2.0),
    frc::Translation2d(-DriveConstants::kWheelBase / 2.0, DriveConstants::kTrackWidth / 2.0),
    frc::Translation2d(-DriveConstants::kWheelBase / 2.0, -DriveConstants::kTrackWidth / 2.0),
  };

  frc::SwerveDriveOdometry<4> m_odometry{
    m_kinematics,
    frc::Rotation2d(units::degree_t{m_gyro.GetYaw()}),
    
  };

  void UpdateOdometry();

  void NormalizeDrive();
};
