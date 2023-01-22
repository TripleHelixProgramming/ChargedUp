#include "util/SimSwerveModule.h"

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/Timer.h>
#include <units/voltage.h>

#include <string>

#include <units/angle.h>
#include <units/time.h>
#include <units/length.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

using namespace std::literals;

frc::SwerveModuleState SimSwerveModule::GetState() {
  Update();
  return {m_wheelSim.GetAngularVelocity() * (ModuleConstants::kWheelRadius / 1_rad), m_steerAngle};
}

frc::SwerveModulePosition SimSwerveModule::GetPosition() {
  Update();
  frc::SmartDashboard::PutNumber("Modules/"s + std::to_string(id) + "/Flywheel Speed", (m_wheelSim.GetAngularVelocity() * (ModuleConstants::kWheelRadius / 1_rad)).value());
  frc::SmartDashboard::PutNumber("Modules/"s + std::to_string(id) + "/Flywheel Position", m_drivePosition.value());
  return {m_drivePosition, m_steerAngle};
}

void SimSwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState) {
  Update();
  m_driveController.SetSetpoint(desiredState.speed.value());
  frc::SmartDashboard::PutNumber("Modules/"s + std::to_string(id) + "/Flywheel Setpoint", desiredState.speed.value());
  m_wheelSim.SetInputVoltage(units::volt_t{m_driveController.Calculate((m_wheelSim.GetAngularVelocity() * (ModuleConstants::kWheelRadius / 1_rad)).value())});
  m_steerAngle = desiredState.angle.Radians();
}

SimSwerveModule::SimSwerveModule(int id)
    : id(id),
    m_wheelSim(frc::DCMotor::NEO(1), ModuleConstants::kDriveGearRatio, 0.003_kg_sq_m),
    m_driveController(frc::SmartDashboard::GetNumber("Flywheel P", 3.4), 0.0, 0.0),
    m_ts(frc::Timer::GetFPGATimestamp()) {
  frc::SmartDashboard::PutNumber("Flywheel P", 3.4);
}

void SimSwerveModule::Update() {
  m_driveController.SetP(frc::SmartDashboard::GetNumber("Flywheel P", 3.4));
  auto dt = frc::Timer::GetFPGATimestamp() - m_ts;
  m_drivePosition += m_wheelSim.GetAngularVelocity() * (ModuleConstants::kWheelRadius / 1_rad) * dt;
  m_wheelSim.Update(dt);
  m_ts += dt;
}
