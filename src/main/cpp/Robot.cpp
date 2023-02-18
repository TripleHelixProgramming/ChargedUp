// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "Robot.hpp"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/TimesliceRobot.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTableInstance.h>

#include "RobotContainer.hpp"

using namespace frc;

Robot::Robot() : frc::TimesliceRobot{2_ms, 5_ms} {
  // Schedule periodic functions
  Schedule(
      [=, this] {
        if (IsEnabled()) {
          m_container.SuperstructurePeriodic();
        }
      },
      1.5_ms);

  if constexpr (RobotBase::IsSimulation()) {
    // auto inst = nt::NetworkTableInstance::GetDefault();
    // inst.StopServer();
    // inst.SetServer("localhost");
    // inst.StartClient4("Robot Simulation");
  }
}

void Robot::RobotInit() {
  // Starts recording to data log
  DataLogManager::Start();

  // Record both DS control and joystick data
  DriverStation::StartDataLog(DataLogManager::GetLog());

  // Initialize the LEDs
  m_leds.SetLength(kLEDBuffLength);
  m_leds.SetData(m_ledBuffer);
  m_leds.Start();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  m_container.UpdateTelemetry();

  // Rainbow();
  // Yellow();
  // Green();
  Purple();
  m_leds.SetData(m_ledBuffer);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  m_container.RunDisabled();
}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

void Robot::Rainbow() {
  // For every pixel
  for (int i = 0; i < kLEDBuffLength; i++) {
    // Calculate the hue - hue is easier for rainbows because the color
    // shape is a circle so only one value needs to precess
    const auto pixelHue = (firstPixelHue + (i * 180 / kLEDBuffLength)) % 180;
    // Set the value
    m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
  }
  // Increase by to make the rainbow "move"
  firstPixelHue += 3;
  // Check bounds
  firstPixelHue %= 180;
}

void Robot::Yellow() {
  for (int i = 0; i < kLEDBuffLength; i++) {
    m_ledBuffer[i].SetRGB(255, 100, 0);
  }
}

void Robot::Green() {
  for (int i = 0; i < kLEDBuffLength; i++) {
    m_ledBuffer[i].SetRGB(0, 255, 0);
  }
}

void Robot::Purple() {
  for (int i = 0; i < kLEDBuffLength; i++) {
    m_ledBuffer[i].SetRGB(150, 0, 255);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
