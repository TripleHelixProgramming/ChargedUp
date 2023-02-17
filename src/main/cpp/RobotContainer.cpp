// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "RobotContainer.hpp"

#include <iostream>

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <wpi/json.h>

#include "Constants.hpp"
#include "commands/DriveTrajectory.hpp"
#include "commands/ResetAbsoluteEncoders.hpp"
#include "commands/autos/North2ConeCharge.h"
#include "util/log/DoubleTelemetryEntry.hpp"

using namespace frc;
using namespace frc2;
using namespace OIConstants;
using namespace units;

RobotContainer::RobotContainer()
    : m_oiDriverLeftXLog("OI/Driver/Left X"),
      m_oiDriverRightXLog("OI/Driver/Right X"),
      m_oiDriverRightYLog("OI/Driver/Right Y") {
  m_drive.SetDefaultCommand(RunCommand(
      [this] {  // onExecute
        // Right stick up on xbox is negative, right stick down is postive.
        // Right stick right on xbox is negative, right stick left is postive.
        // Left stick right is positive, left stick left is negative.
        return m_drive.JoystickDrive(-m_driver.GetRawAxis(kZorroRightYAxis),
                                     -m_driver.GetRawAxis(kZorroRightXAxis),
                                     -m_driver.GetRawAxis(kZorroLeftXAxis),
                                     true);
      },
      {&m_drive}  // requirements
      ));

  ConfigureBindings();

  m_trajManager.LoadTrajectories();

  SmartDashboard::PutData("Reset Encoders",
                          new ResetAbsoluteEncoders(&m_drive));
}

std::optional<CommandPtr> RobotContainer::GetAutonomousCommand() {
  return North2ConeCharge(&m_drive, &m_superstructure, &m_trajManager).ToPtr();
}

void RobotContainer::UpdateTelemetry() {
  m_oiDriverLeftXLog.Append(m_driver.GetRawAxis(kZorroLeftXAxis));
  m_oiDriverRightXLog.Append(m_driver.GetRawAxis(kZorroRightXAxis));
  m_oiDriverRightYLog.Append(m_driver.GetRawAxis(kZorroRightYAxis));
  m_compressor.IsEnabled();
  SmartDashboard::PutNumber("Arm absolute position",
                            m_superstructure.GetAbsoluteArmPosition().value());
  SmartDashboard::PutNumber("Arm relative position",
                            m_superstructure.GetArmPosition().value());
  SmartDashboard::PutNumber("String position",
                            m_superstructure.RawString());
  SmartDashboard::PutNumber("Estimated string position",
                            m_superstructure.GetAbsoluteStringPosition());
  SmartDashboard::PutNumber("Estimated angle",
                            m_superstructure.GetStringAngle().value());
}

void RobotContainer::ConfigureBindings() {
  JoystickButton zorroGIn{&m_driver, kZorroGIn};
  zorroGIn.OnTrue(InstantCommand([this]() {
                    return m_drive.ResetOdometry(Pose2d());
                  }).ToPtr());

  JoystickButton zorroLeftBumper{&m_driver, kZorroAIn};

  zorroLeftBumper.OnTrue((InstantCommand([this]() {
                           return m_superstructure.Outtake();
                         })).ToPtr());
  zorroLeftBumper.OnFalse((InstantCommand([this]() {
                            return m_superstructure.SetIntakeWheelSpeed(0.0);
                          })).ToPtr());

  // m_operator.X().OnTrue(IntakeCone(&m_gripper).ToPtr());
  // m_operator.X().OnFalse(
  //     InstantCommand([this]() { return m_gripper.Retract(); },
  //     {&m_gripper}).ToPtr());
  // m_operator.Y().OnTrue(IntakeCube(&m_gripper).ToPtr());
  // m_operator.Y().OnFalse(
  //     InstantCommand([this]() { return m_gripper.Retract(); },
  //     {&m_gripper}).ToPtr());
  // m_operator.A().OnTrue(
  //     InstantCommand([this]() { return m_gripper.EjectGamePiece(); },
  //     {&m_gripper}).ToPtr());
  // m_operator.A().OnFalse(
  //     InstantCommand([this]() { return m_gripper.Retract(); },
  //     {&m_gripper}).ToPtr());
  m_operator.X().OnTrue(
      (InstantCommand([this]() { m_superstructure.IntakeCone(); })).ToPtr());
  m_operator.X().OnFalse((InstantCommand([this]() {
                           m_superstructure.SetIntakeWheelSpeed(0.0);
                         })).ToPtr());
  m_operator.Y().OnTrue(
      (InstantCommand([this]() { m_superstructure.IntakeCube(); })).ToPtr());
  m_operator.Y().OnFalse((InstantCommand([this]() {
                           m_superstructure.SetIntakeWheelSpeed(0.0);
                         })).ToPtr());
  m_operator.A().OnTrue((InstantCommand([this]() {
                          m_superstructure.PositionConeMedium();
                        })).ToPtr());
  m_operator.B().OnTrue((InstantCommand([this]() {
                          m_superstructure.PositionConeHigh();
                        })).ToPtr());
  m_operator.RightBumper().OnTrue((InstantCommand([this]() {
                                    m_superstructure.SetIntakeWheelSpeed(0.5);
                                  })).ToPtr());
  m_operator.RightBumper().OnFalse(
      (InstantCommand([this]() {
        return m_superstructure.SetIntakeWheelSpeed(0.0);
      })).ToPtr());
  // m_operator.().OnTrue((InstantCommand([this]() { return
  // m_superstructure.SetIntakeWheelSpeed(0.5); })).ToPtr());
  // m_operator.RightBumper().OnFalse((InstantCommand([this]() { return
  // m_superstructure.SetIntakeWheelSpeed(0.0); })).ToPtr());
}

void RobotContainer::RunDisabled() {
  m_superstructure.SyncEncoders();
}

void RobotContainer::SuperstructurePeriodic() {
  m_superstructure.SuperstructurePeriodic();
}
