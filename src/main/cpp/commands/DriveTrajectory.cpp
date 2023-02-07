// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/DriveTrajectory.h"

#include <frc/Timer.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

using namespace units;

DriveTrajectory::DriveTrajectory(SwerveDrive* drive,
                                 const Trajectory* trajectory)
    : m_drive{drive},
      m_trajectory{trajectory},
      m_timestampLog("Trajectory/Timestamp"),
      m_xSetpointLog("Trajectory/X Setpoint"),
      m_ySetpointLog("Trajectory/Y Setpoint"),
      m_thetaSetpointLog("Trajectory/Theta Setpoint") {
  AddRequirements(m_drive);
}

void DriveTrajectory::Initialize() {
  m_timestamp.Reset();
  m_timestamp.Start();
  m_drive->ResetOdometry(m_trajectory->GetInitialPose());
}

void DriveTrajectory::Execute() {
  auto state = m_trajectory->Sample(m_timestamp.Get());
  auto currentPose = m_drive->GetPose();

  m_controllerX.SetSetpoint(state.pose.X().value());
  m_controllerY.SetSetpoint(state.pose.Y().value());
  m_controllerRotation.SetSetpoint(state.pose.Rotation().Radians().value());

  auto vx =
      state.velocity.vx +
      meters_per_second_t{m_controllerX.Calculate(currentPose.X().value())};
  auto vy =
      state.velocity.vy +
      meters_per_second_t{m_controllerY.Calculate(currentPose.Y().value())};
  auto omega = state.velocity.omega +
               radians_per_second_t{m_controllerRotation.Calculate(
                   currentPose.Rotation().Radians().value())};

  m_timestampLog.Append(m_timestamp.Get().value());
  m_xSetpointLog.Append(state.pose.X().value());
  m_ySetpointLog.Append(state.pose.Y().value());
  m_thetaSetpointLog.Append(state.pose.Rotation().Radians().value());

  m_drive->Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      vx, vy, omega, m_drive->GetPose().Rotation()));
}

void DriveTrajectory::End(bool interrupted) {
  m_drive->Brake();
}

bool DriveTrajectory::IsFinished() {
  return m_timestamp.Get() > m_trajectory->GetTotalTime();
}
