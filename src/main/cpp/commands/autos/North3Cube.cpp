// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/North3Cube.hpp"

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

North3Cube::North3Cube(SwerveDrive* drive, Superstructure* superstructure,
                       bool isBlue) {
  std::string allianceSidePrefix =
      isBlue ? "red-" : "red-";  // TODO until we have a blue one
  AddCommands(
      ParallelDeadlineGroup(
          SequentialCommandGroup(
              WaitCommand(0.0_s),
              DriveTrajectory(drive,
                              &TrajectoryManager::GetInstance().GetTrajectory(
                                  allianceSidePrefix + "north-3cube_0_pick4"))),
          SequentialCommandGroup(InstantCommand([superstructure]() {
                                   superstructure->SetIntakeWheelSpeed(-0.5);
                                 }),
                                 WaitCommand(1_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->IntakeCone();
                                 }))),
      DriveTrajectory(drive, &TrajectoryManager::GetInstance().GetTrajectory(
                                 allianceSidePrefix + "north-3cube_1_place8")),
      ParallelDeadlineGroup(
          SequentialCommandGroup(
              WaitCommand(0.1_s),
              DriveTrajectory(drive,
                              &TrajectoryManager::GetInstance().GetTrajectory(
                                  allianceSidePrefix + "north-3cube_2_pick3"))),
          SequentialCommandGroup(
              InstantCommand([superstructure]() { superstructure->Outtake(); }),
              WaitCommand(1_s), InstantCommand([superstructure]() {
                superstructure->IntakeCone();
              }))),
      DriveTrajectory(drive, &TrajectoryManager::GetInstance().GetTrajectory(
                                 allianceSidePrefix + "north-3cube_3_place7")),
      InstantCommand([superstructure]() { superstructure->Outtake(); }),
      ParallelDeadlineGroup(
          DriveTrajectory(drive,
                          &TrajectoryManager::GetInstance().GetTrajectory(
                              allianceSidePrefix + "north-3cube_4_chgst")),
          SequentialCommandGroup(WaitCommand(0.75_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->PositionLow();
                                 })))

  );
}

frc::Pose2d North3Cube::GetStartingPose(bool isBlue) {
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
