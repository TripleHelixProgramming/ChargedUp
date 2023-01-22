#pragma once

#include <units/time.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <AHRS.h>

class SimGyro {
public:
  units::radian_t GetYaw();

  void SetRotationSpeed(units::radians_per_second_t rotationSpeed);

  SimGyro();

 private:
  AHRS m_realGyro{frc::SPI::Port::kMXP};

  void UpdateSim();

  units::radians_per_second_t m_rotationSpeed{0.0_rad_per_s};

  units::second_t m_ts;
  units::radian_t m_gyroYaw{0.0_rad};
};