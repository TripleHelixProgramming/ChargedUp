// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveDrive.h"

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>

#include <units/angle.h>

using namespace frc;
using namespace ElectricalConstants;

SwerveDrive::SwerveDrive() {
  m_gyro.Calibrate();
}

void SwerveDrive::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

frc::Pose2d SwerveDrive::GetPose() const {
  return m_odometry.GetPose();
}

void SwerveDrive::ResetOdometry(const frc::Pose2d& pose) {
  m_odometry.ResetPosition(GetGyroHeading(), 
                           {m_modules[0].GetPosition(), 
                            m_modules[1].GetPosition(), 
                            m_modules[2].GetPosition(),
                            m_modules[3].GetPosition()},
                           pose);
}

Rotation2d SwerveDrive::GetGyroHeading() {
  auto rawGyro = units::radian_t{m_gyro.GetYaw()} - fieldRelativeAngleOffset.Radians(); // Returns yaw as -180 to +180
  float calc_yaw = raw_yaw;

  if (0.0 > raw_yaw ) { // yaw is negative
    calc_yaw += 360.0;
  }
  return Rotation2d.fromDegrees(-calc_yaw);
  return Rotation2d(degree_t{AHRS.GetYaw()});
}

void SwerveDrive::UpdateOdometry() {

}

void SwerveDrive::NormalizeDrive(const std::array<frc::SwerveModuleState, 4>& desiredStates, const frc::ChassisSpeeds speeds) {

}