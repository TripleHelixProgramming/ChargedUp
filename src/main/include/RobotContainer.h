// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Joystick.h>

#include "Constants.h"
#include "subsystems/SwerveDrive.h"
#include "subsystems/SwerveModule.h"

class RobotContainer {
 public:
  RobotContainer();

 private:
  SwerveDrive m_drive;

  frc::Joystick m_driver{0};
  frc::Joystick m_operator{1};

  void ConfigureBindings();

  frc2::CommandPtr GetAutonomousCommand();
};
