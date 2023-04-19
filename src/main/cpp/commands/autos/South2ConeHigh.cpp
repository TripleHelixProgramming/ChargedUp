// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/South2ConeHigh.hpp"

#include <string>

#include <frc/geometry/Pose2d.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/DriveTrajectory.hpp"
#include "subsystems/Superstructure.hpp"

using namespace frc2;

South2ConeHigh::South2ConeHigh(SwerveDrive* drive,
                               Superstructure* superstructure, bool isBlue) {
  std::string allianceSidePrefix = isBlue ? "blue-" : "red-";
  AddCommands(
      InstantCommand([superstructure]() { superstructure->PositionHigh(); }),
      WaitCommand(0.9_s),
      DriveTrajectory(drive,
                      &TrajectoryManager::GetTrajectory(
                          allianceSidePrefix + "south-2cone-high_0_place1")),
      InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),
      WaitCommand(0.1_s),

      ParallelDeadlineGroup(
          DriveTrajectory(drive,
                          &TrajectoryManager::GetTrajectory(
                              allianceSidePrefix + "south-2cone-high_1_pick1")),
          SequentialCommandGroup(WaitCommand(0.5_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->IntakeCube();
                                   superstructure->m_stealth = true;
                                 })),
          SequentialCommandGroup(
              WaitCommand(TrajectoryManager::GetTrajectory(
                              allianceSidePrefix + "south-2cone-high_1_pick1")
                              .GetTotalTime() -
                          0.2_s),
              InstantCommand(
                  [superstructure]() { superstructure->IntakeCone(); }),
              WaitCommand(0.1_s), InstantCommand([superstructure]() {
                superstructure->m_stealth = false;
              }))),
      ParallelDeadlineGroup(
          DriveTrajectory(
              drive, &TrajectoryManager::GetTrajectory(
                         allianceSidePrefix + "south-2cone-high_2_place3")),
          SequentialCommandGroup(WaitCommand(3.8_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->PositionHigh();
                                 }))),
      InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }));
}

frc::Pose2d South2ConeHigh::GetStartingPose(bool isBlue) {
  static auto blueStartingPose =
      TrajectoryManager::GetTrajectory("blue-south-2cone-high_0_place1")
          .GetInitialPose();
  static auto redStartingPose =
      TrajectoryManager::GetTrajectory("red-south-2cone-high_0_place1")
          .GetInitialPose();
  if (isBlue)
    return blueStartingPose;
  else
    return redStartingPose;
}
