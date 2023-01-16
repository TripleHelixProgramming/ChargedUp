#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc/geometry/Pose2d.h>

#include "Constants.h"

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

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   return CommandPtr(&Command());
// }
