// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/North2ConeChgstat.hpp"

#include <string>

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/DriveTrajectory.hpp"
#include "subsystems/Superstructure.hpp"

using namespace frc2;

North2ConeChgstat::North2ConeChgstat(SwerveDrive* drive,
                                     Superstructure* superstructure,
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

      DriveTrajectory(drive,
                      &TrajectoryManager::GetTrajectory(
                          allianceSidePrefix + "north-2cone-chgstat_2_align7")),
      ParallelDeadlineGroup(
          DriveTrajectory(
              drive, &TrajectoryManager::GetTrajectory(
                         allianceSidePrefix + "north-2cone-chgstat_3_place7")),
          SequentialCommandGroup(WaitCommand(0.1_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->PositionHigh();
                                 }))),
      InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),

      ParallelDeadlineGroup(
          DriveTrajectory(
              drive,
              &TrajectoryManager::GetTrajectory(
                  allianceSidePrefix + "north-2cone-chgstat_4_chgstat"),
              false),
          SequentialCommandGroup(WaitCommand(0.25_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->IntakeCone();
                                 }))),
      RunCommand(
          [drive]() {
            drive->Drive(frc::ChassisSpeeds{0_mps, 0_mps, 0.01_rad_per_s});
          },
          {drive}));
}

frc::Pose2d North2ConeChgstat::GetStartingPose(bool isBlue) {
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
