#pragma once

#include <frc/simulation/FlywheelSim.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/PIDController.h>
#include <frc/system/plant/DCMotor.h>
#include "Constants.h"

class SimSwerveModule {
 public:

  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& desiredState);

  SimSwerveModule(int id);

 private:
  
  int id;

  void Update();

  units::radian_t m_steerAngle{0.0_rad};

  frc::sim::FlywheelSim m_wheelSim;

  frc::PIDController m_driveController;

  units::second_t m_ts;
  units::meter_t m_drivePosition{0.0_m};
};