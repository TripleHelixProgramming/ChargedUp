// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <array>

#include <AHRS.h>
#include <Constants.h>
#include <frc/StateSpaceUtil.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <units/angle.h>

#include "subsystems/SwerveModule.h"
#include "subsystems/Vision.h"
#include "util/SimGyro.h"

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
  void Periodic() override;

  /**
   * This is a temporary method for testing pose estimation
   */
  void PrintPoseEstimate();

 private:
  // Subsystems:
  /**
   * @brief The four swerve modules.
   */
  std::array<SwerveModule, 4> m_modules;
  Vision m_vision;

  // Properties:
  /**
   * @brief NAVX gyro sensor for heading
   */
  SimGyro m_gyro;
  photonlib::PhotonCamera m_camera{"OV5647"};

  /**
   * @brief Swerve drive kinematics
   */
  frc::SwerveDriveKinematics<4> m_driveKinematics;

  // State:
  /**
   * @brief The current position state of the robot.
   */
  frc::SwerveDriveOdometry<4> m_odometry;

  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  frc::Field2d m_field;
};
