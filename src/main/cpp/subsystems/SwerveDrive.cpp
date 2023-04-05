// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/SwerveDrive.hpp"

#include <cmath>
#include <string>

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
#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Transform3d.h"
#include "networktables/NetworkTable.h"
#include "photonlib/PhotonPoseEstimator.h"
#include "util/log/DoubleTelemetryEntry.hpp"
#include "util/log/TelemetryEntry.hpp"

using namespace frc;
using namespace frc2;
using namespace photonlib;
using namespace units;

using namespace DriveConstants;
using namespace ElectricalConstants;
using namespace ModuleConstants;

using namespace std::string_literals;

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
                 Rotation2d(GetGyroHeading()),
                 {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
                  m_modules[2].GetPosition(), m_modules[3].GetPosition()},
                 Pose2d()},
      m_poseEstimator{m_driveKinematics,
                      Rotation2d(GetGyroHeading()),
                      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
                       m_modules[2].GetPosition(), m_modules[3].GetPosition()},
                      Pose2d(),
                      {0.1, 0.1, 0.1},
                      {0.1, 0.1, 0.1}},
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
                                double joystickRotate, bool fieldRelative,
                                bool isBlue) {
  ChassisSpeeds speeds =
      fieldRelative
          ? ChassisSpeeds::FromFieldRelativeSpeeds(
                joystickDrive * kMaxVelocityX, joystickStrafe * kMaxVelocityY,
                joystickRotate * kMaxVelocityAngular,
                m_poseEstimator.GetEstimatedPosition().Rotation().RotateBy(
                    Rotation2d{isBlue ? 0_deg : 180_deg}))
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

bool IsVisionPoseValid(const Pose3d& pose) {
  bool isOk = units::math::abs(pose.Z()) < 20_cm && (pose.X() > 11.5_m || pose.X() < 5_m);
  static int countOk = 0;
  static int countTot = 0;
  if (isOk) {
    ++countOk;
  }
  ++countTot;
  SmartDashboard::PutNumber("Vision/Percentage Valid",
                            (double) countOk / countTot);
  return isOk;
}

void ApplyVisionResult(std::optional<photonlib::EstimatedRobotPose> result,
                       const Transform3d& transform,
                       second_t& lastTs, 
                       Field2d& visionField,
                       SwerveDrivePoseEstimator<4>& poseEstimator,
                       const char* cameraName) {
  if (result.has_value()) {
    Pose3d pose = result->estimatedPose.TransformBy(transform.Inverse());
    SmartDashboard::PutNumber("Vision/"s + cameraName + " pose X", pose.X().value());
    SmartDashboard::PutNumber("Vision/"s + cameraName + " pose Y", pose.Y().value());
    SmartDashboard::PutNumber("Vision/"s + cameraName + " pose Z", pose.Z().value());
    SmartDashboard::PutNumber("Vision/"s + cameraName + " pose X rot", pose.Rotation().X().value());
    SmartDashboard::PutNumber("Vision/"s + cameraName + " pose Y rot", pose.Rotation().Y().value());
    SmartDashboard::PutNumber("Vision/"s + cameraName + " pose Z rot", pose.Rotation().Z().value());
    if (!IsVisionPoseValid(pose)) {
      return;
    }
    second_t timestamp = result->timestamp;
    if (timestamp > lastTs) {
      SmartDashboard::PutNumber("Vision/"s + cameraName + " latency", (Timer::GetFPGATimestamp() - timestamp).value());
      poseEstimator.AddVisionMeasurement(pose.ToPose2d(), timestamp);
      lastTs = timestamp;
      visionField.SetRobotPose(pose.ToPose2d());
    }
  }
}

void SwerveDrive::Periodic() {
  m_odometry.Update(  // TODO: Remove odometry
      GetGyroHeading(),
      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
       m_modules[2].GetPosition(), m_modules[3].GetPosition()});
  m_poseEstimator.UpdateWithTime(
      Timer::GetFPGATimestamp(),
      GetGyroHeading(),
      {m_modules[0].GetPosition(), m_modules[1].GetPosition(),
       m_modules[2].GetPosition(), m_modules[3].GetPosition()});

  ApplyVisionResult(m_leftCamera.GetResult(), VisionConstants::kRobotToLeftCam, m_lastLeftAppliedTs, m_visionEstField, m_poseEstimator, "Left");
  ApplyVisionResult(m_rightCamera.GetResult(), VisionConstants::kRobotToRightCam, m_lastRightAppliedTs, m_visionEstField, m_poseEstimator, "Right");
  ApplyVisionResult(m_rearCamera.GetResult(), VisionConstants::kRobotToBackCam, m_lastRearAppliedTs, m_visionEstField, m_poseEstimator, "Rear");

  auto pose = m_poseEstimator.GetEstimatedPosition();

  SmartDashboard::PutNumber("Drive/Pose Estimate/X", pose.X().value());
  SmartDashboard::PutNumber("Drive/Pose Estimate/Y", pose.Y().value());
  SmartDashboard::PutNumber("Drive/Pose Estimate/Theta", pose.Rotation().Radians().value());

  m_poseEstField.SetRobotPose(pose);
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

void SwerveDrive::SyncAbsoluteEncoders() {
  for (auto& _module : m_modules) {
    _module.SyncEncoders();
  }
}

wpi::array<SwerveModulePosition, 4> SwerveDrive::GetModulePositions() const {
  wpi::array<SwerveModulePosition, 4> modulePositions(wpi::empty_array);
  for (size_t modIdx = 0; modIdx < 4; modIdx++) {
    modulePositions[modIdx] = m_modules[modIdx].GetPosition();
  }
  return modulePositions;
}
