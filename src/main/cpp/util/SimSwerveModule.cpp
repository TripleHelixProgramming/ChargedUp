#include "util/SimSwerveModule.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/Timer.h>

#include <units/angle.h>
#include <units/time.h>
#include <units/length.h>
#include <frc/kinematics/SwerveModulePosition.h>

frc::SwerveModuleState SimSwerveModule::GetState() {
  Update();
  return m_state;
}

frc::SwerveModulePosition SimSwerveModule::GetPosition() {
  Update();
  return {m_drivePosition, m_state.angle};
}

void SimSwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState) {
  Update();
  m_state = desiredState;
}

SimSwerveModule::SimSwerveModule()
  : m_state{0_mps, 0_rad},
  m_ts(frc::Timer::GetFPGATimestamp()),
  m_drivePosition(0_m) {}

void SimSwerveModule::Update() {
  auto dt = frc::Timer::GetFPGATimestamp() - m_ts;
  m_drivePosition += m_state.speed * dt;
  m_ts += dt;
}