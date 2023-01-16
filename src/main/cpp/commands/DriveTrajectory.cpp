// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// #include "commands/DriveTrajectory.h"

// #include <frc/Timer.h>

// using namespace frc;

// DriveTrajectory::DriveTrajectory(SwerveDrive* drive, Trajectory& trajectory) : 
//     m_drive{drive}, m_trajectory{trajectory} {
//   AddRequirements(drive);
// }

// void DriveTrajectory::Initialize() {
//   m_timer.Reset();
//   m_timer.Start();
// }

// void DriveTrajectory::Execute() {
//   auto state = m_trajectory.Sample(m_timer.Get());

//   auto vx = state.vx;
//   auto vy = state.vy;
//   auto omega = state.omega;

//   m_drive->Drive(ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, m_drive->GetPose().Rotation()));
// }

// void DriveTrajectory::End(bool interrupted) {
//   m_drive->Brake();
// }

// bool DriveTrajectory::IsFinished() {
//   return m_timer.Get() < m_trajectory.GetTotalTime();
// }
