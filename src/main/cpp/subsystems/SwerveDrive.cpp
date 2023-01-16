#include "subsystems/SwerveDrive.h"

#include <cmath>

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <units/angle.h>
#include <units/base.h>
#include <units/time.h>

using namespace frc;
using namespace units;
using namespace ElectricalConstants;
using namespace DriveConstants;
using namespace ModuleConstants;

SwerveDrive::SwerveDrive()
  : m_modules{{
    SwerveModule(kDriveMotorPorts[0], kSteerMotorPorts[0], kAbsEncoderPorts[0]), 
    SwerveModule(kDriveMotorPorts[1], kSteerMotorPorts[1], kAbsEncoderPorts[1]),
    SwerveModule(kDriveMotorPorts[2], kSteerMotorPorts[2], kAbsEncoderPorts[2]),
    SwerveModule(kDriveMotorPorts[3], kSteerMotorPorts[3], kAbsEncoderPorts[3])
  }},
  m_driveKinematics{{
    frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}
  }},
  m_odometry{m_driveKinematics,
             Rotation2d(units::degree_t{m_gyro.GetYaw()}),
             {m_modules[0].GetPosition(), m_modules[1].GetPosition(), 
              m_modules[2].GetPosition(), m_modules[3].GetPosition()}, 
             Pose2d()} {
}

Pose2d SwerveDrive::GetPose() const {
  return m_odometry.GetPose();
}

void SwerveDrive::ResetOdometry(const Pose2d& pose) {
  m_odometry.ResetPosition(Rotation2d(units::degree_t{m_gyro.GetYaw()}), 
                           {m_modules[0].GetPosition(), 
                            m_modules[1].GetPosition(), 
                            m_modules[2].GetPosition(),
                            m_modules[3].GetPosition()},
                            pose);
}

void SwerveDrive::JoystickDrive(double joystickDrive, 
                                double joystickStrafe,
                                double joystickRotate, 
                                bool fieldRelative) {
  ChassisSpeeds speeds = fieldRelative ? 
      ChassisSpeeds::FromFieldRelativeSpeeds(joystickDrive * kMaxVelocityX, 
                                             joystickStrafe * kMaxVelocityY, 
                                             joystickRotate * kMaxVelocityAngular,
                                             GetPose().Rotation()) : 
      ChassisSpeeds{joystickDrive * kMaxVelocityX,
                    joystickStrafe * kMaxVelocityY, 
                    joystickRotate * kMaxVelocityAngular};
  auto states = m_driveKinematics.ToSwerveModuleStates(speeds);

  SmartDashboard::PutNumber("vx: ", speeds.vx.value());
  SmartDashboard::PutNumber("vy: ", speeds.vy.value());
  SmartDashboard::PutNumber("omega: ", speeds.omega.value());

  // use most extreme axis as scale factor
  double scale = std::max({joystickDrive, joystickStrafe, joystickRotate});

  // identify fastest motor's speed
  auto largestWheelSpeed = 0.0_mps;
  for (auto& moduleState : states) {
    largestWheelSpeed = meters_per_second_t{std::max(largestWheelSpeed.value(), std::abs(moduleState.speed.value()))};
  }

  if (largestWheelSpeed.value() != 0.0 && (largestWheelSpeed / scale).value() > kMaxSpeed.value()) {
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
}

void SwerveDrive::Brake() {
  for (auto& module : m_modules) {
    module.SetDesiredState(SwerveModuleState{0.0_mps, module.GetState().angle});
  }
}

void SwerveDrive::Periodic() {
  m_odometry.Update(Rotation2d(units::degree_t{m_gyro.GetYaw()}), 
                    {m_modules[0].GetPosition(), m_modules[1].GetPosition(), 
                     m_modules[2].GetPosition(), m_modules[3].GetPosition()});
}
