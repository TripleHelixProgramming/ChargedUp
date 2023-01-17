#include "RobotContainer.h"

#include <iostream>

#include <wpi/json.h>

#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/button/JoystickButton.h>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>


#include "Constants.h"
#include "commands/DriveTrajectory.h"

using namespace frc;
using namespace frc2;
using namespace OIConstants;

RobotContainer::RobotContainer() {
  m_drive.SetDefaultCommand(RunCommand(
    [this] {                           // onExecute
      return m_drive.JoystickDrive(m_driver.GetRawAxis(kZorroRightYAxis), 
                                   m_driver.GetRawAxis(kZorroRightXAxis), 
                                   -m_driver.GetRawAxis(kZorroLeftXAxis), 
                                   true);
    },
    {&m_drive} // requirements
  ));

  ConfigureBindings();

  m_trajManager.LoadTrajectories();
}

void RobotContainer::ConfigureBindings() {
  JoystickButton zorroGIn{&m_driver, kZorroGIn};
  zorroGIn.OnTrue(InstantCommand([this]() { return m_drive.ResetOdometry(Pose2d()); }).ToPtr());
  
  m_operator.X().OnTrue(InstantCommand([this]() { return m_gripper.Extend(); }).ToPtr());
  m_operator.Y().OnTrue(InstantCommand([this]() { return m_gripper.Retract(); }).ToPtr());

  m_operator.A().OnTrue(InstantCommand([this]() { return m_gripper.SetWheelSpeeds(0.5); }).ToPtr());
  m_operator.B().OnTrue(InstantCommand([this]() { return m_gripper.SetWheelSpeeds(-0.5); }).ToPtr());
  m_operator.A().OnFalse(InstantCommand([this]() { return m_gripper.SetWheelSpeeds(0.0); }).ToPtr());
  m_operator.B().OnFalse(InstantCommand([this]() { return m_gripper.SetWheelSpeeds(0.0); }).ToPtr());
}

std::optional<CommandPtr> RobotContainer::GetAutonomousCommand() {
  // SmartDashboard::PutNumber("Traj Total Time", m_trajManager.GetTrajectory("traj").GetTotalTime().value());
  return CommandPtr(DriveTrajectory(&m_drive, m_trajManager.GetTrajectory("traj")));
  // return std::nullopt;
}
