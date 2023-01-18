// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <optional>

#include <frc/Joystick.h>

#include "Constants.h"
#include "subsystems/Gripper.h"
#include "subsystems/SwerveDrive.h"
#include "util/TrajectoryManager.h"

class RobotContainer {
 public:
  RobotContainer();

  std::optional<frc2::CommandPtr> GetAutonomousCommand();

 private:
  // Subsystems
  SwerveDrive m_drive;
  Gripper m_gripper;

  // Operator Interface (OI)
  frc::Joystick m_driver{0};
  frc2::CommandXboxController m_operator{1};

  TrajectoryManager m_trajManager;

  void ConfigureBindings();
};
