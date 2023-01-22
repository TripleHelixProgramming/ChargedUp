#pragma once

#include <frc/simulation/FlywheelSim.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/kinematics/SwerveModulePosition.h>

class SimSwerveModule {
 public:
  // void Update();

  // units::meter_t GetDrivePosition() const;
  // units::radian_t GetSteerAngle() const;

  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& desiredState);

  SimSwerveModule();

 private:

  void Update();

  frc::SwerveModuleState m_state;

  // frc::sim::FlywheelSim m_wheelSim;

  units::second_t m_ts;
  units::meter_t m_drivePosition;
};