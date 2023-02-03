// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <array>

#include <AHRS.h>
#include <Constants.h>
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

#include "subsystems/SwerveModule.h"
#include "subsystems/Vision.h"
#include "util/log/DoubleTelemetryEntry.h"

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

  void SimulationPeriodic() override;

  void ResetAbsoluteEncoders();

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
  AHRS m_gyro{frc::SPI::Port::kMXP};
  photonlib::PhotonCamera m_camera{"front"};

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

  DoubleTelemetryEntry m_poseEstimateXLog;
  DoubleTelemetryEntry m_poseEstimateYLog;
  DoubleTelemetryEntry m_poseEstimateThetaLog;

  DoubleTelemetryEntry m_visionPoseEstimateXLog;
  DoubleTelemetryEntry m_visionPoseEstimateYLog;
  DoubleTelemetryEntry m_visionPoseEstimateThetaLog;

  frc::Field2d m_field;

  // Simulation
  // frc::Timer m_simTimer;

  frc::sim::SimDeviceSim m_gyroSim;
  hal::SimDouble m_gyroSimYaw;

  std::array<frc::SwerveModulePosition, 4> m_previousModulePositions{};

};
