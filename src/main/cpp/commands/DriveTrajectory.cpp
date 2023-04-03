// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/DriveTrajectory.hpp"

#include <frc/Timer.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

using namespace units;

DriveTrajectory::DriveTrajectory(SwerveDrive* drive,
                                 const Trajectory* trajectory, bool useVision, bool seedInitialPose)
    : m_drive{drive},
      m_trajectory{trajectory},
      m_useVision(useVision),
      m_seedInitialPose(seedInitialPose),
      m_timestampLog("Trajectory/Timestamp"),
      m_xSetpointLog("Trajectory/X Setpoint"),
      m_ySetpointLog("Trajectory/Y Setpoint"),
      m_thetaSetpointLog("Trajectory/Theta Setpoint") {
  AddRequirements(m_drive);
}

void DriveTrajectory::Initialize() {
  m_timestamp.Reset();
  m_timestamp.Start();

  m_controllerRotation.EnableContinuousInput(-std::numbers::pi,
                                             std::numbers::pi);

  if (!m_useVision || m_seedInitialPose) {
    m_drive->ResetOdometry(m_trajectory->GetInitialPose());
  }
}

void DriveTrajectory::Execute() {
  auto state = m_trajectory->Sample(m_timestamp.Get());
  auto currentPose =
      m_useVision ? m_drive->GetPose() : m_drive->GetOdometryPose();

  m_controllerX.SetSetpoint(state.pose.X().value());
  m_controllerY.SetSetpoint(state.pose.Y().value());
  m_controllerRotation.SetSetpoint(state.pose.Rotation().Radians().value());

  auto vx = state.vx + meters_per_second_t{
                           m_controllerX.Calculate(currentPose.X().value())};
  auto vy = state.vy + meters_per_second_t{
                           m_controllerY.Calculate(currentPose.Y().value())};
  auto omega =
      state.omega + radians_per_second_t{m_controllerRotation.Calculate(
                        currentPose.Rotation().Radians().value())};

  m_timestampLog.Append(m_timestamp.Get().value());
  m_xSetpointLog.Append(state.pose.X().value());
  m_ySetpointLog.Append(state.pose.Y().value());
  m_thetaSetpointLog.Append(state.pose.Rotation().Radians().value());

  m_drive->Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      vx, vy, omega, currentPose.Rotation()));
}

void DriveTrajectory::End(bool interrupted) {
  m_drive->ResetOdometry(
      m_trajectory->Sample(m_trajectory->GetTotalTime()).pose);
  m_drive->Brake();
}

bool DriveTrajectory::IsFinished() {
  return m_timestamp.Get() > m_trajectory->GetTotalTime();
}
