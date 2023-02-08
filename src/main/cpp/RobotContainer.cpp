// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "RobotContainer.h"

#include <iostream>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <wpi/json.h>

#include "Constants.h"
#include "commands/DriveTrajectory.h"
#include "commands/ResetAbsoluteEncoders.h"
#include "util/log/DoubleTelemetryEntry.h"
#include "util/trajectory/TrajectoryConfig.h"
#include "util/trajectory/TrajectoryGenerator.h"
#include "util/trajectory/constraint/MaxAccelerationConstraint.h"
#include "util/trajectory/constraint/MaxVelocityConstraint.h"
#include "util/trajectory/constraint/TrajectoryConstraint.h"

using namespace frc;
using namespace frc2;
using namespace OIConstants;

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
  Pose2d startPose(0_m, 0_m, 0_rad);
  Pose2d endPose(1_m, 0_m, 0_rad);

  TrajectoryConfig config;
  auto maxVelocityConstraint = MaxVelocityConstraint(units::meters_per_second_t{1.0},
                                                     units::meters_per_second_t{1.0},
                                                     units::radians_per_second_t{1.0});
  auto maxAccelerationConstraint = MaxAccelerationConstraint(1.0,
                                                             1.0,
                                                             1.0);
  config.ApplyConstraint(maxVelocityConstraint);
  config.ApplyConstraint(maxAccelerationConstraint);

  TrajectoryGenerator generator{config};


  wpi::json j = generator.Generate(startPose, endPose);
  std::cout << j.dump() << std::endl;


  return CommandPtr(
      DriveTrajectory(&m_drive, &m_trajManager.GetTrajectory("traj")));
}

void RobotContainer::UpdateTelemetry() {
  m_oiDriverLeftXLog.Append(m_driver.GetRawAxis(kZorroLeftXAxis));
  m_oiDriverRightXLog.Append(m_driver.GetRawAxis(kZorroRightXAxis));
  m_oiDriverRightYLog.Append(m_driver.GetRawAxis(kZorroRightYAxis));
}

void RobotContainer::ConfigureBindings() {
  JoystickButton zorroGIn{&m_driver, kZorroGIn};
  zorroGIn.OnTrue(InstantCommand([this]() {
                    return m_drive.ResetOdometry(Pose2d());
                  }).ToPtr());

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
}

void RobotContainer::SuperstructurePeriodic() {
  m_superstructure.SuperstructurePeriodic();
}
