// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "rev/SparkMaxLimitSwitch.h"

class Superstructure : public frc2::SubsystemBase {
 public:
  Superstructure();

  void IntakeCube();
  void IntakeCone();
  void EjectGamePiece();
  void Retract();

  bool BeamBreakTriggered();

  void Periodic() override;

 private:
  rev::CANSparkMax m_leftWheel{15, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightWheel{16, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_arm{4, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_armFollower{5, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxLimitSwitch m_beamBreak;

  frc::DoubleSolenoid m_expander{frc::PneumaticsModuleType::CTREPCM, 0, 1};
  frc::DoubleSolenoid m_lifter{frc::PneumaticsModuleType::CTREPCM, 2, 3};
};
