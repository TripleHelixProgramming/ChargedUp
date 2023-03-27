// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/TestAuto.hpp"

#include <exception>
#include <string>

#include <frc/geometry/Pose2d.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/AutoBalance.hpp"
#include "commands/DriveTrajectory.hpp"
#include "subsystems/Superstructure.hpp"

TestAuto::TestAuto(SwerveDrive* drive, Superstructure* superstructure) {
  using namespace frc2;
  AddCommands(
      frc2::InstantCommand(
            [superstructure]() { superstructure->PositionHigh(); }),
      frc2::WaitCommand(0.6_s),
      DriveTrajectory(
              drive, TrajectoryManager::GetInstance().GetTrajectory(
                        "red-north-3piece-0_place8")),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),
      frc2::ParallelDeadlineGroup(
          DriveTrajectory(
              drive, TrajectoryManager::GetInstance().GetTrajectory(
                         "red-north-3piece-1_pick4")),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.35_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->IntakeCone();
                                       }))),
      frc2::ParallelDeadlineGroup(
          SequentialCommandGroup(
            DriveTrajectory(
              drive, TrajectoryManager::GetInstance().GetTrajectory(
                        "red-north-3piece-2_align9")),
            DriveTrajectory(
              drive, TrajectoryManager::GetInstance().GetTrajectory(
                        "red-north-3piece-2_place9"))
          ),
          frc2::SequentialCommandGroup(frc2::WaitCommand(1_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->PositionHigh();
                                       }))
      ),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); })
  );
}

frc::Pose2d TestAuto::GetStartingPose(bool isBlue) {
  static auto blueStartingPose =
      TrajectoryManager::GetInstance()
          .GetTrajectory("red-north-3piece-0_place8")
          .GetInitialPose();
  static auto redStartingPose =
      TrajectoryManager::GetInstance()
          .GetTrajectory("red-north-3piece-0_place8")
          .GetInitialPose();
  if (isBlue)
    return blueStartingPose;
  else
    return redStartingPose;
}
