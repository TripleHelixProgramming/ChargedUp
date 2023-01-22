#include "util/SimGyro.h"

#include <units/time.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <frc/RobotBase.h>
#include <frc/Timer.h>

#include <AHRS.h>

units::radian_t SimGyro::GetYaw() {
  if (frc::RobotBase::IsSimulation()) {
    UpdateSim();
    return m_gyroYaw;
  }
  return units::degree_t{-m_realGyro.GetYaw()};
}

void SimGyro::SetRotationSpeed(units::radians_per_second_t rotationSpeed) {
  if (frc::RobotBase::IsSimulation()) {
    UpdateSim();
    m_rotationSpeed = rotationSpeed;
  }
}

SimGyro::SimGyro()
  : m_ts{frc::Timer::GetFPGATimestamp()} {}

void SimGyro::UpdateSim() {
  auto dt = frc::Timer::GetFPGATimestamp() - m_ts;
  m_gyroYaw += m_rotationSpeed * dt;
  m_ts += dt;
}