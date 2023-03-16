// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/Timer.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxLimitSwitch.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include "Constants.hpp"
#include "util/log/BoolTelemetryEntry.hpp"
#include "util/log/DoubleTelemetryEntry.hpp"

class Superstructure : public frc2::SubsystemBase {
 public:
  Superstructure();

  void PositionHigh();

  void PositionMedium();

  void PositionLow();

  void IntakeCube();

  void IntakeCone();

  void IntakeCubeStation();

  void IntakeConeStation();

  void SetIntakeWheelSpeed(double speed);

  void SetExtenderPosition(bool extender);

  void SyncEncoders();

  void Outtake();

  void SetArmPosition(units::radian_t position);

  units::degree_t GetAbsoluteArmPosition();

  double GetAbsoluteStringPosition();

  units::degree_t RawPosition();

  double GetRelativePosition();

  units::degree_t GetStringAngle();

  double RawString();

  bool HasGamePiece();

  void SuperstructurePeriodic();

  // Modes for picking up tipped cones
  bool m_flipConeMode = false;
  bool m_flipConeUp = false;
  bool m_expanded = true;
  bool m_stealth = false;

 private:
  // State variables
  double m_intakeWheelSpeed = 0.0;
  units::degree_t m_armPosition = 0.0_rad;

  double m_integral = 0.0;
  double m_kI = 0.002;
  double m_tolerance = 4.0;
  double m_outputBound = 0.0;

  double m_seed = 0.0;

  bool m_lastBeamBreakDetection = false;

  // Strint pot lookup table
  std::array<double, 13> m_encoderPositions{0.835, 2.315,  4.275, 7.05,   10.60,
                                            13.97, 17.345, 20.71, 23.667, 27.45,
                                            31.53, 35.12,  39.85};
  std::array<double, 13> m_stringPositions{
      0.0,     148.0,   318.25, 533.0,   847.0,   1134.0, 1401.5,
      1650.25, 1829.75, 2023.0, 2230.75, 2358.25, 2466.0};

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

  frc::Encoder m_relativeEncoder{1, 2, false, frc::Encoder::EncodingType::k4X};

  frc::Encoder m_stringEncoder{3, 4, false, frc::Encoder::EncodingType::k4X};

  rev::SparkMaxLimitSwitch m_beamBreak;

  frc::DoubleSolenoid m_expander{frc::PneumaticsModuleType::REVPH, 0,
                                 1};  // TODO should be in constants
  frc::DoubleSolenoid m_intakePop{frc::PneumaticsModuleType::REVPH, 2, 3};
  frc::Timer m_intakePopTimer;

  // Logging
  DoubleTelemetryEntry m_intakeWheelSpeedLog{"Intake/Wheel Speed"};
  BoolTelemetryEntry m_intakeExpandedLog{"Intake/Is Expanded"};
  DoubleTelemetryEntry m_armLeftCurrentLog{"Arm/Left Current"};
  DoubleTelemetryEntry m_armRightCurrentLog{"Arm/Right Current"};
  DoubleTelemetryEntry m_armStateErrorLog{"Arm/State Error"};
  DoubleTelemetryEntry m_armTargetAngleLog{"Arm/Target Angle (deg)"};
  DoubleTelemetryEntry m_armAppliedVoltageLog{"Arm/Applied Voltage"};
};
