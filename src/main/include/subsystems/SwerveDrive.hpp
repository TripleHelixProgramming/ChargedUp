// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <array>

#include <AHRS.h>
#include <Constants.hpp>
#include <frc/StateSpaceUtil.h>
#include <frc/Timer.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <units/angle.h>

#include "subsystems/SwerveModule.hpp"
#include "util/SimSwervePoseTracker.hpp"
#include "util/cadmia/CadmiaCamera.hpp"
#include "util/log/DoubleTelemetryEntry.hpp"

class SwerveDrive : public frc2::SubsystemBase {
 public:
  SwerveDrive();

  /**
   * Get the vision and odometry fused pose.
   */
  frc::Pose2d GetPose() const;

  /**
   * Get the pose only calculated based on odometry.
   */
  frc::Pose2d GetOdometryPose() const;

  frc::Rotation2d GetGyroHeading();

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
   * Update pose of simulated robot based on motor speeds.
   */
  void SimulationPeriodic() override;

  /**
   * Zero steering absolute encoders for all swerve modules, and store the
   * offsets in the encoders' memories.
   */
  void ResetAbsoluteEncoders();

 private:
  /**
   * Get the positions of all swerve modules, as an array.
   */
  wpi::array<frc::SwerveModulePosition, 4> GetModulePositions() const;

  // Subsystems:
  /// The four swerve modules.
  std::array<SwerveModule, 4> m_modules;
  
  // Properties:
  /**
   * @brief NAVX gyro sensor for heading
   */
  AHRS m_gyro{frc::SPI::Port::kMXP};

  frc::Rotation2d angle;

  double lastAngle;

  /// The camera facing forwar
  cadmia::CadmiaCamera m_rightCamera{"video2"};
  cadmia::CadmiaCamera m_rearCamera{"video1"};
  cadmia::CadmiaCamera m_leftCamera{"video0"};

  /// Swerve drive kinematics
  frc::SwerveDriveKinematics<4> m_driveKinematics;

  // State:
  /// The position of the robot as estimated by module encoders
  frc::SwerveDriveOdometry<4> m_odometry;

  /// The position of the robot based on odometry and vision measurements
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  units::second_t m_lastLeftAppliedTs{0};
  units::second_t m_lastRightAppliedTs{0};
  units::second_t m_lastRearAppliedTs{0};

  // Logging
  DoubleTelemetryEntry m_poseEstimateXLog;
  DoubleTelemetryEntry m_poseEstimateYLog;
  DoubleTelemetryEntry m_poseEstimateThetaLog;

  DoubleTelemetryEntry m_visionPoseEstimateXLog;
  DoubleTelemetryEntry m_visionPoseEstimateYLog;
  DoubleTelemetryEntry m_visionPoseEstimateThetaLog;

  /// Display pose estimate on 2d field
  frc::Field2d m_poseEstField;
  frc::Field2d m_visionEstField;

  // Simulation

  /// Keeps track of the simulated robot's pose
  SimSwervePoseTracker<4> m_simPoseTracker;

  /// Simulates the gyro sensor
  frc::sim::SimDeviceSim m_gyroSim;

  /// Display sim pose on 2d field
  frc::Field2d m_simPoseField;
};
