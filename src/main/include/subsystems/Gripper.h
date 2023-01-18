// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>

class Gripper : public frc2::SubsystemBase {
 public:
  Gripper();

  void SetWheelSpeeds(double power);

  void Extend();
  void Retract();

 private:
  rev::CANSparkMax m_leftWheel{15, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightWheel{16, rev::CANSparkMax::MotorType::kBrushless};

  frc::DoubleSolenoid m_expander{frc::PneumaticsModuleType::CTREPCM, 0, 1};
};
