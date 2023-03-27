// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/AutoBalance.hpp"

#include <frc/Timer.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace units;

AutoBalance::AutoBalance(SwerveDrive* drive)
    : m_drive{drive} {
  AddRequirements(m_drive);
  timer.Start();
}

void AutoBalance::Initialize() {
  timer.Reset();
  m_startTime = timer.Get().value();

  m_lastAngle = 0.0;
  m_lastTime = 0.0;
  m_tippedTime = 0.0;
  m_startTime = 0.0;

  tipped = false;
  end = false;
}

void AutoBalance::Execute() {
  auto robotPose = m_drive->GetPose();
  auto rotation = m_drive->GetGyroRotation();

  double roll = rotation.Y().value();
  double time = timer.Get().value();
  double rate = (roll - m_lastAngle) / (time - m_lastTime);
  m_lastAngle = roll;
  m_lastTime = time;

  frc::SmartDashboard::PutNumber("Gyro/Roll Rate", rate);
  
  if (!tipped) {
    if (robotPose.X() > 13.5_m) {
      m_drive->Drive(frc::ChassisSpeeds{-2.0_mps, 0.0_mps, 0.0_rad_per_s});
    } else if (units::math::abs(rotation.Y()) < 8_deg) {
      tipped = true;
      m_tippedTime = time;
    } else if (rotation.Y() < 0.0_deg) {
      m_drive->Drive(frc::ChassisSpeeds{-0.9_mps, 0.0_mps, 0.0_rad_per_s});
    } else {
      tipped = true;
      m_tippedTime = time;
    }
  } else {
    if (time - m_tippedTime < 0.4) {
      m_drive->Drive(frc::ChassisSpeeds{1.4_mps, 0.0_mps, 0.0_rad_per_s});
    } else {
      frc::SmartDashboard::PutNumber("Balance time", timer.Get().value() - m_startTime);
      m_drive->Drive(frc::ChassisSpeeds{0_mps, 0_mps, 0.01_rad_per_s});
      end = true;
    }
  }
  
}

void AutoBalance::End(bool interrupted) {
  m_drive->Drive(frc::ChassisSpeeds{0_mps, 0_mps, 0.01_rad_per_s});
}

bool AutoBalance::IsFinished() {
  return end;
}
