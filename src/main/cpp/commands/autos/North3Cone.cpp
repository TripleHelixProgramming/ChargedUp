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

North3Cone::North3Cone(SwerveDrive* drive,
                                     Superstructure* superstructure,
                                     bool isBlue) {
  std::string allianceSidePrefix = isBlue ? "red-" : "red-"; // TODO: <--- for testing only
  AddCommands(
      frc2::InstantCommand(
          [superstructure]() { superstructure->PositionHigh(); }),
      frc2::WaitCommand(0.9_s),
      DriveTrajectory(drive,
                      &TrajectoryManager::GetInstance().GetTrajectory(
                          allianceSidePrefix + "north-2cone-chgstat_0_place9")),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),
      frc2::WaitCommand(0.1_s),

      frc2::ParallelDeadlineGroup(
          DriveTrajectory(
              drive, &TrajectoryManager::GetInstance().GetTrajectory(
                         allianceSidePrefix + "north-2cone-chgstat_1_pick4")),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.25_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->IntakeCone();
                                       }))),

      DriveTrajectory(drive,
                      &TrajectoryManager::GetInstance().GetTrajectory(
                          allianceSidePrefix + "north-3cone_2_align7")),
      frc2::ParallelDeadlineGroup(
          DriveTrajectory(
              drive, &TrajectoryManager::GetInstance().GetTrajectory(
                         allianceSidePrefix + "north-3cone_3_place7")),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.1_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->PositionHigh();
                                       }))),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),

      frc2::ParallelDeadlineGroup(
          DriveTrajectory(
              drive,
              &TrajectoryManager::GetInstance().GetTrajectory(
                  allianceSidePrefix + "north-3cone_4_pick3"),
              false),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.25_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->IntakeCone();
                                       }))));
}

frc::Pose2d North3Cone::GetStartingPose(bool isBlue) {
  static auto blueStartingPose =
      TrajectoryManager::GetInstance()
          .GetTrajectory("blue-north-2cone-chgstat_0_place9")
          .GetInitialPose();
  static auto redStartingPose =
      TrajectoryManager::GetInstance()
          .GetTrajectory("red-north-2cone-chgstat_0_place9")
          .GetInitialPose();
  if (isBlue)
    return blueStartingPose;
  else
    return redStartingPose;
}
