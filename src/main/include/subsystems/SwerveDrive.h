// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <subsystems/SwerveModule.h>
#include <Constants.h>
#include <array>
#include <AHRS.h>
#include <units/angle.h>

class SwerveDrive : public frc2::SubsystemBase {
 public:
  SwerveDrive();

  frc::Pose2d GetPose() const;

  void ResetOdometry(const frc::Pose2d& pose);

  /**
   * Get the heading of the robot from the gyro.
   */
  frc::Rotation2d GetHeading();

  /**
   * Zero the heading (GetHeading() will return 0.0 after).
   */
  void ZeroHeading();
  
  /**
   * @brief Command the robot to drive with the given joystick inputs.
   */
  void JoystickDrive(double joystickDrive, double joystickStrafe, double joystickRotate, bool fieldRelative);

  /**
   * @brief Command the robot to drive with the given speeds.
   */
  void Drive(const frc::ChassisSpeeds& speeds, bool fieldRelative);

  /**
   * @brief Command the robot to brake, setting all motors to brake mode.
   */
  void Brake();

  /**
   * 
   */
  void Periodic() override;

 private:
  // Subsystems:
  /**
   * The four swerve modules.
   */
  std::array<SwerveModule, 4> m_modules;

  /**
   * Calculates sweve module states from 
   */
  frc::SwerveDriveKinematics<4> m_kinematics;

  // State:
  /**
   * @brief The current position state of the robot.
   */
  frc::SwerveDriveOdometry<4> m_odometry;

  /**
   * @brief The current angular offset between the field coordinate system
   * and the robot coordinate system.
   */
  frc::Rotation2d m_fieldRelativeAngleOffset;

  // Properties:
  /**
   * NAVX gyro sensor for heading
   */
  AHRS m_gyro{frc::SPI::Port::kMXP};
};
