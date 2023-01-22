// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/DriveTrajectory.h"

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

using namespace units;

DriveTrajectory::DriveTrajectory(SwerveDrive* drive, Trajectory& trajectory)
    : m_drive{drive}, m_trajectory{trajectory} {
  AddRequirements(drive);
}

void DriveTrajectory::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_drive->ResetOdometry(m_trajectory.GetInitialPose());
}

void DriveTrajectory::Execute() {
  auto state = m_trajectory.Sample(m_timer.Get());
  auto currentPose = m_drive->GetPose();

  m_controllerX.SetSetpoint(state.pose.X().value());
  m_controllerY.SetSetpoint(state.pose.Y().value());
  m_controllerRotation.SetSetpoint(state.pose.Rotation().Radians().value());

  auto vx = state.vx +
            meters_per_second_t{m_controllerX.Calculate(currentPose.X().value())};
  auto vy = state.vy +
            meters_per_second_t{m_controllerY.Calculate(currentPose.Y().value())};
  auto omega = state.omega + radians_per_second_t{m_controllerRotation.Calculate(
                                 currentPose.Rotation().Radians().value())};

  frc::SmartDashboard::PutNumber("Target x: ", state.pose.X().value());
  frc::SmartDashboard::PutNumber("Target y: ", state.pose.Y().value());
  frc::SmartDashboard::PutNumber("Target rotation: ",
                            state.pose.Rotation().Radians().value());

  frc::SmartDashboard::PutNumber("Real x: ", currentPose.X().value());
  frc::SmartDashboard::PutNumber("Real y: ", currentPose.Y().value());
  frc::SmartDashboard::PutNumber("Real rotation: ",
                            currentPose.Rotation().Radians().value());

  m_drive->Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      vx, vy, omega, m_drive->GetPose().Rotation()));
}

void DriveTrajectory::End(bool interrupted) {
  m_drive->Brake();
}

bool DriveTrajectory::IsFinished() {
  return m_timer.Get() > m_trajectory.GetTotalTime();
}
