// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <subsystems/SwerveModule.h>

#include <array>

#include <AHRS.h>
#include <Constants.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>

class SwerveDrive : public frc2::SubsystemBase {
 public:
  SwerveDrive();

  frc::Pose2d GetPose() const;

  void ResetOdometry(const frc::Pose2d& pose);

  /**
   * @brief Command the robot to drive with the given joystick inputs.
   * Each input is a value from -1 to 1, like
   */
  void JoystickDrive(double joystickDrive, double joystickStrafe,
                     double joystickRotate, bool fieldRelative);

  /**
   * @brief Command the robot to drive with the given speeds.
   */
  void Drive(const frc::ChassisSpeeds& speeds);

  /**
   * @brief Command the robot to brake, setting all motors to brake mode.
   */
  void Brake();

  /**
   * Update the current pose of the robot based on a combination of
   * gyro, vision, and encoder sensor data.
   */
  virtual void Periodic() override;

 private:
  // Subsystems:
  /**
   * @brief The four swerve modules.
   */
  std::array<SwerveModule, 4> m_modules;

  // Properties:
  /**
   * @brief NAVX gyro sensor for heading
   */
  AHRS m_gyro{frc::SPI::Port::kMXP};

  /**
   * @brief Swerve drive kinematics
   */
  frc::SwerveDriveKinematics<4> m_driveKinematics;

  // State:
  /**
   * @brief The current position state of the robot.
   */
  frc::SwerveDriveOdometry<4> m_odometry;
};
