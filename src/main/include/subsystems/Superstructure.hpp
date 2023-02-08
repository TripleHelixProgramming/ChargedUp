// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxLimitSwitch.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include "Constants.hpp"

class Superstructure : public frc2::SubsystemBase {
 public:
  Superstructure();

  void IntakeCube();

  void IntakeCone();

  void SetIntakeWheelSpeed(double speed);

  void SetExtenderPosition(bool extender);

  void SyncEncoders();

  void Outtake();

  void SetArmPosition(units::radian_t position);

  units::degree_t GetAbsoluteArmPosition();

  units::degree_t GetArmPosition();

  bool HasGamePiece();

  void SuperstructurePeriodic();

 private:
  // State variables
  double m_intakeWheelSpeed = 0.0;
  units::degree_t m_armPosition = 0.0_rad;
  bool m_expanded = true;

  double m_integral = 0.0;
  double m_kI = 0.01;
  double m_tolerance = 10.0;
  double m_outputBound = 0.0;

  units::degree_t m_armOffset = units::degree_t{0.0};

  // Arm PID controller
  frc::ProfiledPIDController<units::radian> m_armController{
      SuperstructureConstants::kArmP, SuperstructureConstants::kArmI,
      SuperstructureConstants::kArmD,
      frc::TrapezoidProfile<units::radian>::Constraints{10.0_rad_per_s,
                                                        7.5_rad_per_s / 1.0_s},
      5_ms};

  // Hardware modules
  rev::CANSparkMax m_leftWheel{ElectricalConstants::kIntakeLeftWheelPort,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_rightWheel{ElectricalConstants::kIntakeRightWheelPort,
                                rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax m_armLeader{ElectricalConstants::kArmLeaderPort,
                               rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_armFollower{ElectricalConstants::kArmFollowerPort,
                                 rev::CANSparkMax::MotorType::kBrushless};

  frc::DutyCycleEncoder m_armEncoder{ElectricalConstants::kArmEncoderPort};
  frc::Encoder m_armRelativeEncoder{1, 2, false,
                                    frc::Encoder::EncodingType::k4X};

  rev::SparkMaxLimitSwitch m_beamBreak;

  frc::DoubleSolenoid m_expander{frc::PneumaticsModuleType::REVPH, 0, 1};
};