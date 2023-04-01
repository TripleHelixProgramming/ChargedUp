// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/North3Cone.hpp"

#include <string>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/DriveTrajectory.hpp"
#include "subsystems/Superstructure.hpp"

using namespace frc2;

North3Cone::North3Cone(SwerveDrive* drive, Superstructure* superstructure,
                       bool isBlue) {
  std::string allianceSidePrefix = isBlue ? "blue-" : "red-";
  AddCommands(
      InstantCommand([superstructure]() { superstructure->PositionHigh(); }),
      WaitCommand(0.9_s),
      DriveTrajectory(drive,
                      &TrajectoryManager::GetTrajectory(
                          allianceSidePrefix + "north-2cone-chgstat_0_place9")),
      InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),
      WaitCommand(0.1_s),

      ParallelDeadlineGroup(
          DriveTrajectory(
              drive, &TrajectoryManager::GetTrajectory(
                         allianceSidePrefix + "north-2cone-chgstat_1_pick4")),
          SequentialCommandGroup(WaitCommand(0.25_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->IntakeCone();
                                 }))),

      DriveTrajectory(drive, &TrajectoryManager::GetTrajectory(
                                 allianceSidePrefix + "north-3cone_2_align7")),
      ParallelDeadlineGroup(
          DriveTrajectory(
              drive, &TrajectoryManager::GetTrajectory(allianceSidePrefix +
                                                       "north-3cone_3_place7")),
          SequentialCommandGroup(WaitCommand(0.1_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->PositionHigh();
                                 }))),
      InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),

      ParallelDeadlineGroup(
          DriveTrajectory(drive,
                          &TrajectoryManager::GetTrajectory(
                              allianceSidePrefix + "north-3cone_4_pick3"),
                          false),
          SequentialCommandGroup(WaitCommand(0.5_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->IntakeCone();
                                 }))));
}

frc::Pose2d North3Cone::GetStartingPose(bool isBlue) {
  static auto blueStartingPose =
      TrajectoryManager::GetTrajectory("blue-north-2cone-chgstat_0_place9")
          .GetInitialPose();
  static auto redStartingPose =
      TrajectoryManager::GetTrajectory("red-north-2cone-chgstat_0_place9")
          .GetInitialPose();
  if (isBlue)
    return blueStartingPose;
  else
    return redStartingPose;
}
