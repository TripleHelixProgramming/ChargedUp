// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/SwerveDrive.h"

#include <cmath>

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/RobotBase.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <photonlib/PhotonCamera.h>
#include <units/angle.h>
#include <units/base.h>
#include <units/time.h>
#include <units/length.h>

#include <frc/kinematics/SwerveModulePosition.h>
#include "frc/RobotBase.h"
#include "subsystems/Vision.h"

using namespace frc;
using namespace photonlib;
using namespace units;

using namespace DriveConstants;
using namespace ElectricalConstants;
using namespace ModuleConstants;

SwerveDrive::SwerveDrive()
    : m_modules{{SwerveModule(kDriveMotorPorts[0], kSteerMotorPorts[0],
                              kAbsEncoderPorts[0]),
                 SwerveModule(kDriveMotorPorts[1], kSteerMotorPorts[1],
                              kAbsEncoderPorts[1]),
                 SwerveModule(kDriveMotorPorts[2], kSteerMotorPorts[2],
                              kAbsEncoderPorts[2]),
                 SwerveModule(kDriveMotorPorts[3], kSteerMotorPorts[3],
                              kAbsEncoderPorts[3])}},
      m_driveKinematics{
          {frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
           frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
           frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
           frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}}},
      m_odometry{m_driveKinematics,
                 Rotation2d(m_gyro.GetYaw()),
                 {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
                  m_modules[2].GetPosition(), m_modules[3].GetPosition()},
                 Pose2d()},
      m_poseEstimator{m_driveKinematics,
                 Rotation2d(m_gyro.GetYaw()),
                 {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
                  m_modules[2].GetPosition(), m_modules[3].GetPosition()},
                 Pose2d(),
                 {0.1, 0.1, 0.1},
                 {0.1, 0.1, 0.1}} {
  SmartDashboard::PutData("Field", &m_field);
}

Pose2d SwerveDrive::GetPose() const {
  return m_odometry.GetPose();
}

void SwerveDrive::ResetOdometry(const Pose2d& pose) {
  m_odometry.ResetPosition(
      Rotation2d(m_gyro.GetYaw()),
      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
       m_modules[2].GetPosition(), m_modules[3].GetPosition()},
      pose);
  m_poseEstimator.ResetPosition(
      Rotation2d(m_gyro.GetYaw()),
      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
       m_modules[2].GetPosition(), m_modules[3].GetPosition()},
      pose);
}

void SwerveDrive::JoystickDrive(double joystickDrive, double joystickStrafe,
                                double joystickRotate, bool fieldRelative) {
  ChassisSpeeds speeds =
      fieldRelative
          ? ChassisSpeeds::FromFieldRelativeSpeeds(
                joystickDrive * kMaxVelocityX, joystickStrafe * kMaxVelocityY,
                joystickRotate * kMaxVelocityAngular, GetPose().Rotation())
          : ChassisSpeeds{joystickDrive * kMaxVelocityX,
                          joystickStrafe * kMaxVelocityY,
                          joystickRotate * kMaxVelocityAngular};
  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

  SmartDashboard::PutNumber("Drive/Commanded Velocity/vx", speeds.vx.value());
  SmartDashboard::PutNumber("Drive/Commanded Velocity/vy", speeds.vy.value());
  SmartDashboard::PutNumber("Drive/Commanded Velocity/omega", speeds.omega.value());

  // use most extreme axis as scale factor
  double scale = std::max({joystickDrive, joystickStrafe, joystickRotate});

  // identify fastest motor's speed
  auto largestWheelSpeed = 0.0_mps;
  for (auto& moduleState : states) {
    largestWheelSpeed = meters_per_second_t{std::max(
        largestWheelSpeed.value(), std::abs(moduleState.speed.value()))};
  }

  if (largestWheelSpeed.value() != 0.0 &&
      (largestWheelSpeed / scale).value() > kMaxSpeed.value()) {
    for (auto moduleState : states) {
      moduleState.speed *= scale * kMaxSpeed / largestWheelSpeed;
    }
  }

  for (size_t i = 0; i < states.size(); ++i) {
    m_modules[i].SetDesiredState(states[i]);
  }
}

void SwerveDrive::Drive(const frc::ChassisSpeeds& speeds) {
  auto moduleStates = m_driveKinematics.ToSwerveModuleStates(speeds);
  for (size_t i = 0; i < moduleStates.size(); ++i) {
    m_modules[i].SetDesiredState(moduleStates[i]);
  }

  m_gyro.SetRotationSpeed(m_driveKinematics.ToChassisSpeeds(moduleStates).omega);
}

void SwerveDrive::Brake() {
  for (auto& module : m_modules) {
    module.SetDesiredState(SwerveModuleState{0.0_mps, module.GetState().angle});
  }
}

void SwerveDrive::Periodic() {
  m_odometry.Update(Rotation2d(m_gyro.GetYaw()),
                    {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
                     m_modules[2].GetPosition(), m_modules[3].GetPosition()});

  m_field.SetRobotPose(m_odometry.GetPose());

  m_gyro.SetRotationSpeed(m_driveKinematics.ToChassisSpeeds(
      m_modules[0].GetState(),
      m_modules[1].GetState(),
      m_modules[2].GetState(),
      m_modules[3].GetState()).omega);

  m_poseEstimator.Update(
      Rotation2d(m_gyro.GetYaw()),
      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
       m_modules[2].GetPosition(), m_modules[3].GetPosition()});
  auto visionEstimatedPose = m_vision.GetEstimatedGlobalPose(
      Pose3d(m_poseEstimator.GetEstimatedPosition()));
  SmartDashboard::PutNumber("Drive/Pose Estimate/X", m_poseEstimator.GetEstimatedPosition().X().value());
  SmartDashboard::PutNumber("Drive/Pose Estimate/Y", m_poseEstimator.GetEstimatedPosition().Y().value());
  SmartDashboard::PutNumber("Drive/Pose Estimate/Theta", m_poseEstimator.GetEstimatedPosition().Rotation().Radians().value());
  if (visionEstimatedPose.has_value()) {
    m_poseEstimator.AddVisionMeasurement(
        visionEstimatedPose->estimatedPose.ToPose2d(),
        visionEstimatedPose->timestamp);
    SmartDashboard::PutNumber("Vision/Pose Estimate/X", visionEstimatedPose->estimatedPose.X().value());
    SmartDashboard::PutNumber("Vision/Pose Estimate/Y", visionEstimatedPose->estimatedPose.Y().value());
    SmartDashboard::PutNumber("Vision/Pose Estimate/Theta", visionEstimatedPose->estimatedPose.ToPose2d().Rotation().Radians().value());
  }



  PrintPoseEstimate();
}

void SwerveDrive::PrintPoseEstimate() {
  auto result = m_camera.GetLatestResult();
  bool hasTargets = result.HasTargets();
  SmartDashboard::PutBoolean("Vision/Has Targets", hasTargets);
  if (!hasTargets)
    return;
  auto target = result.GetBestTarget();
  int targetID = target.GetFiducialId();
  double poseAmbiguity = target.GetPoseAmbiguity();
  SmartDashboard::PutNumber("Vision/Target ID", targetID);
  SmartDashboard::PutNumber("Vision/Pose Ambiguity", poseAmbiguity);

  
}
