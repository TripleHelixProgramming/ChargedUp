// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxLimitSwitch.h>
#include <units/angle.h>

#include "Constants.h"
#include "frc/DutyCycleEncoder.h"

using namespace ElectricalConstants;

class Superstructure : public frc2::SubsystemBase {
 public:
  Superstructure();

  void SetIntakeWheelSpeed(double speed);

  void SetArmPosition(units::radian_t position);

  units::radian_t GetArmPosition();

  bool HasGamePiece();

  void SuperstructurePeriodic();

 private:
  // State variables
  double m_intakeWheelSpeed = 0.0;
  units::radian_t m_armPosition = 0.0_rad;
  bool m_expanded = false;

  // Arm PID controller
  frc2::PIDController m_armController{SuperstructureConstants::kArmP,
                                      SuperstructureConstants::kArmI,
                                      SuperstructureConstants::kArmD};

  // Hardware modules
  rev::CANSparkMax m_leftWheel{15, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightWheel{16, rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_armLeader{kArmLeaderPort, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_armFollower{kArmFollowerPort, rev::CANSparkMax::MotorType::kBrushless};

  frc::DutyCycleEncoder m_armEncoder{kArmEncoderPort};

  rev::SparkMaxLimitSwitch m_beamBreak;

  frc::DoubleSolenoid m_expander{frc::PneumaticsModuleType::CTREPCM, 0, 1};
  frc::DoubleSolenoid m_lifter{frc::PneumaticsModuleType::CTREPCM, 2, 3};
};
