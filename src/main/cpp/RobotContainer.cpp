#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc/geometry/Pose2d.h>

#include "Constants.h"

using namespace frc;
using namespace frc2;
using namespace OIConstants;

RobotContainer::RobotContainer() {
  m_drive.SetDefaultCommand(FunctionalCommand(
    [] {},                         // onInit
    [this] {                           // onExecute
      return m_drive.JoystickDrive(m_driver.GetRawAxis(kXboxRightYAxis), 
                                   m_driver.GetRawAxis(kXboxRightXAxis), 
                                   -m_driver.GetRawAxis(kXboxLeftXAxis), 
                                   true);
    },
    [] (bool interrupted) {},      // onEnd
    [] { return false; },          // isFinished
    {&m_drive} // requirements
  ));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  JoystickButton A{&m_driver, kXboxA};

  A.WhenPressed(InstantCommand([this]() { return m_drive.ResetOdometry(Pose2d()); }));
}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   return CommandPtr(&Command());
// }
