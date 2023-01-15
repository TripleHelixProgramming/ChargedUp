#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/FunctionalCommand.h>

#include "Constants.h"

using namespace frc;
using namespace frc2;
using namespace OIConstants;

RobotContainer::RobotContainer() {
  m_drive.SetDefaultCommand(FunctionalCommand(
    [this] {},                         // onInit
    [this] {                           // onExecute
      return m_drive.JoystickDrive(m_driver.GetRawAxis(kXboxRightYAxis), 
                                   m_driver.GetRawAxis(kXboxRightXAxis), 
                                   m_driver.GetRawAxis(kXboxLeftXAxis), 
                                   true);
    },
    [this] (bool interrupted) {},      // onEnd
    [this] { return false; },          // isFinished
    {&m_drive} // requirements
  ));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   return CommandPtr(&Command());
// }
