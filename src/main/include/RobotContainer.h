// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/Gripper.h"
#include "subsystems/SwerveDrive.h"

class RobotContainer {
 public:
  RobotContainer();

 private:
  // Subsystems
  SwerveDrive m_drive;
  Gripper m_gripper;

  // Operator Interface (OI)
  frc::Joystick m_driver{0};
  frc2::CommandXboxController m_operator{1};

  void ConfigureBindings();

  // frc2::CommandPtr GetAutonomousCommand();
};
