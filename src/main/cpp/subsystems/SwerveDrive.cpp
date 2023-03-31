// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/SwerveDrive.hpp"

#include <cmath>

#include <Eigen/src/Core/Matrix.h>
#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Twist2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <hal/SimDevice.h>
#include <photonlib/PhotonCamera.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/base.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/array.h>

#include "Constants.hpp"
#include "frc/DriverStation.h"
#include "util/log/DoubleTelemetryEntry.hpp"
#include "util/log/TelemetryEntry.hpp"

using namespace frc;
using namespace frc2;
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
      m_driveKinematics{{
          Translation2d{kWheelBase / 2, kTrackWidth / 2},
          Translation2d{kWheelBase / 2, -kTrackWidth / 2},
          Translation2d{-kWheelBase / 2, kTrackWidth / 2},
          Translation2d{-kWheelBase / 2, -kTrackWidth / 2}
          //  Translation2d{8_in, 13_in},
          //  Translation2d{kWheelBase / 2, -kTrackWidth / 2},
          //  Translation2d{-kWheelBase / 2, kTrackWidth / 2},
          //  Translation2d{-kWheelBase / 2, -kTrackWidth / 2}
      }},
      m_odometry{m_driveKinematics,
                 Rotation2d(units::degree_t{-m_gyro.GetYaw()}),
                 {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
                  m_modules[2].GetPosition(), m_modules[3].GetPosition()},
                 Pose2d()},
      m_poseEstimator{m_driveKinematics,
                      Rotation2d(units::degree_t{-m_gyro.GetYaw()}),
                      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
                       m_modules[2].GetPosition(), m_modules[3].GetPosition()},
                      Pose2d(),
                      {0.1, 0.1, 0.1},
                      {0.3, 0.3, 0.3}},
      m_poseEstimateXLog("Drive/Pose Estimate/X", TelemetryLevel::kCompetition),
      m_poseEstimateYLog("Drive/Pose Estimate/Y", TelemetryLevel::kCompetition),
      m_poseEstimateThetaLog("Drive/Pose Estimate/Theta",
                             TelemetryLevel::kCompetition),
      m_visionPoseEstimateXLog("Vision/Pose Estimate/X",
                               TelemetryLevel::kCompetition),
      m_visionPoseEstimateYLog("Vision/Pose Estimate/Y",
                               TelemetryLevel::kCompetition),
      m_visionPoseEstimateThetaLog("Vision/Pose Estimate/Theta",
                                   TelemetryLevel::kCompetition),
      m_simPoseTracker(m_driveKinematics),
      m_gyroSim("navX-Sensor", 4) {
  SmartDashboard::PutData("Pose Est Field", &m_poseEstField);
  SmartDashboard::PutData("Vision Est Field", &m_visionEstField);
  if constexpr (RobotBase::IsSimulation()) {
    SmartDashboard::PutData("Sim Field", &m_simPoseField);
  }

  m_gyro.Calibrate();

  angle = Rotation2d(degree_t{-m_gyro.GetYaw()});
  lastAngle = -m_gyro.GetYaw();
}

Pose2d SwerveDrive::GetPose() const {
  return m_poseEstimator.GetEstimatedPosition();
}

Pose2d SwerveDrive::GetOdometryPose() const {
  return m_odometry.GetPose();
}

Rotation2d SwerveDrive::GetGyroHeading() {
  double newAngle = -m_gyro.GetYaw();
  double delta =
      std::fmod(std::fmod((newAngle - lastAngle + 180), 360) + 360, 360) -
      180;  // NOLINT
  lastAngle = newAngle;
  angle = angle + Rotation2d(degree_t{delta * 1.02466666667});
  SmartDashboard::PutNumber("Raw angle", newAngle);
  SmartDashboard::PutNumber("Fused angle", -m_gyro.GetFusedHeading());
  SmartDashboard::PutNumber("Angle", angle.Degrees().value());
  return angle;
}

void SwerveDrive::ResetOdometry(const Pose2d& pose) {
  m_odometry.ResetPosition(
      GetGyroHeading(),
      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
       m_modules[2].GetPosition(), m_modules[3].GetPosition()},
      pose);
  m_poseEstimator.ResetPosition(
      GetGyroHeading(),
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
                joystickRotate * kMaxVelocityAngular,
                m_odometry.GetPose().Rotation())
          : ChassisSpeeds{joystickDrive * kMaxVelocityX,
                          joystickStrafe * kMaxVelocityY,
                          joystickRotate * kMaxVelocityAngular};
  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

  // use most extreme axis as scale factor
  double scale = std::max({joystickDrive, joystickStrafe, joystickRotate});

  // desaturate wheel speeds
  // first identify fastest motor's speed
  auto largestWheelSpeed = 0.0_mps;
  for (auto& moduleState : states) {
    largestWheelSpeed = meters_per_second_t{std::max(
        largestWheelSpeed.value(), std::abs(moduleState.speed.value()))};
  }

  // scale speeds
  if (largestWheelSpeed.value() != 0.0 &&
      (largestWheelSpeed / scale).value() > kMaxSpeed.value()) {
    for (auto moduleState : states) {
      moduleState.speed *= scale * kMaxSpeed / largestWheelSpeed;
    }
  }

  // apply setpoints
  for (size_t i = 0; i < states.size(); ++i) {
    m_modules[i].SetDesiredState(states[i]);
  }
}

void SwerveDrive::Drive(const ChassisSpeeds& speeds) {
  auto moduleStates = m_driveKinematics.ToSwerveModuleStates(speeds);
  for (size_t i = 0; i < moduleStates.size(); ++i) {
    m_modules[i].SetDesiredState(moduleStates[i]);
  }
}

void SwerveDrive::Brake() {
  for (auto& module : m_modules) {
    module.SetDesiredState(SwerveModuleState{0.0_mps, module.GetState().angle});
  }
}

void SwerveDrive::Periodic() {
  m_odometry.Update(  // TODO: Remove odometry
      GetGyroHeading(),
      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
       m_modules[2].GetPosition(), m_modules[3].GetPosition()});
  m_poseEstimator.Update(
      GetGyroHeading(),
      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
       m_modules[2].GetPosition(), m_modules[3].GetPosition()});

  auto leftVisionResult = m_leftCamera.GetResult();
  if (leftVisionResult.has_value()) {
    Pose3d pose = leftVisionResult.value().estimatedPose.TransformBy(VisionConstants::kRobotToLeftCam.Inverse());
    auto flatpose = pose.ToPose2d();
    SmartDashboard::PutNumber("/CadmiaLeft/Translation/X", pose.X().value());
    SmartDashboard::PutNumber("/CadmiaLeft/Translation/Y", pose.Y().value());
    SmartDashboard::PutNumber("/CadmiaLeft/Translation/Z", pose.Z().value());
    SmartDashboard::PutNumber("/CadmiaLeft/Rotation/X", pose.Rotation().X().value());
    SmartDashboard::PutNumber("/CadmiaLeft/Rotation/Y", pose.Rotation().Y().value());
    SmartDashboard::PutNumber("/CadmiaLeft/Rotation/Z", pose.Rotation().Z().value());
    second_t timestamp = leftVisionResult.value().timestamp;
    if (timestamp != m_lastLeftAppliedTs) {
      m_poseEstimator.AddVisionMeasurement(flatpose, timestamp);
      m_lastLeftAppliedTs = timestamp;
      m_visionEstField.SetRobotPose(flatpose);
    }
  }

  auto rightVisionResult = m_rightCamera.GetResult();
  if (rightVisionResult.has_value()) {
    Pose3d pose = rightVisionResult.value().estimatedPose.TransformBy(VisionConstants::kRobotToRightCam.Inverse());
    auto flatpose = pose.ToPose2d();
    SmartDashboard::PutNumber("/CadmiaRight/Translation/X", pose.X().value());
    SmartDashboard::PutNumber("/CadmiaRight/Translation/Y", pose.Y().value());
    SmartDashboard::PutNumber("/CadmiaRight/Translation/Z", pose.Z().value());
    SmartDashboard::PutNumber("/CadmiaRight/Rotation/X", pose.Rotation().X().value());
    SmartDashboard::PutNumber("/CadmiaRight/Rotation/Y", pose.Rotation().Y().value());
    SmartDashboard::PutNumber("/CadmiaRight/Rotation/Z", pose.Rotation().Z().value());
    second_t timestamp = rightVisionResult.value().timestamp;
    if (timestamp != m_lastRightAppliedTs) {
      m_poseEstimator.AddVisionMeasurement(flatpose, timestamp);
      m_lastRightAppliedTs = timestamp;
      m_visionEstField.SetRobotPose(flatpose);
    }
  }

  auto rearVisionResult = m_rearCamera.GetResult();
  if (rearVisionResult.has_value()) {
    Pose3d pose = rearVisionResult.value().estimatedPose.TransformBy(VisionConstants::kRobotToBackCam.Inverse());
    auto flatpose = pose.ToPose2d();
    SmartDashboard::PutNumber("/CadmiaRear/Translation/X", pose.X().value());
    SmartDashboard::PutNumber("/CadmiaRear/Translation/Y", pose.Y().value());
    SmartDashboard::PutNumber("/CadmiaRear/Translation/Z", pose.Z().value());
    SmartDashboard::PutNumber("/CadmiaRear/Rotation/X", pose.Rotation().X().value());
    SmartDashboard::PutNumber("/CadmiaRear/Rotation/Y", pose.Rotation().Y().value());
    SmartDashboard::PutNumber("/CadmiaRear/Rotation/Z", pose.Rotation().Z().value());
    second_t timestamp = rearVisionResult.value().timestamp;
    if (timestamp != m_lastRearAppliedTs) {
      m_poseEstimator.AddVisionMeasurement(flatpose, timestamp);
      m_lastRearAppliedTs = timestamp;
      m_visionEstField.SetRobotPose(flatpose);
    }
  }

  SmartDashboard::PutNumber("/Drive/Pose Estimate/X", m_poseEstimator.GetEstimatedPosition().X().value());
  SmartDashboard::PutNumber("/Drive/Pose Estimate/Y", m_poseEstimator.GetEstimatedPosition().Y().value());
  SmartDashboard::PutNumber("/Drive/Pose Estimate/Theta", m_poseEstimator.GetEstimatedPosition().Rotation().Radians().value());

  m_poseEstField.SetRobotPose(m_poseEstimator.GetEstimatedPosition());
}

void SwerveDrive::SimulationPeriodic() {
  Pose2d previousPose = m_simPoseTracker.GetPose();

  m_simPoseTracker.Update(GetModulePositions());

  hal::SimDouble gyroSimYaw = m_gyroSim.GetDouble("Yaw");

  auto deltaRotation =
      m_simPoseTracker.GetPose().Rotation() - previousPose.Rotation();
  degree_t gyroDelta = -deltaRotation.Degrees();
  gyroSimYaw.Set(gyroSimYaw.Get() + gyroDelta.value());

  m_simPoseField.SetRobotPose(m_simPoseTracker.GetPose());
}

void SwerveDrive::ResetAbsoluteEncoders() {
  for (auto& _module : m_modules) {
    _module.ResetEncoders();
  }
}

wpi::array<SwerveModulePosition, 4> SwerveDrive::GetModulePositions() const {
  wpi::array<SwerveModulePosition, 4> modulePositions(wpi::empty_array);
  for (size_t modIdx = 0; modIdx < 4; modIdx++) {
    modulePositions[modIdx] = m_modules[modIdx].GetPosition();
  }
  return modulePositions;
}
