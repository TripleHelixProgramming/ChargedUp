// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/South2Cone.hpp"

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

South2Cone::South2Cone(SwerveDrive* drive, Superstructure* superstructure,
                       bool isBlue) {
  std::string allianceSidePrefix = isBlue ? "blue-" : "red-";
  AddCommands(
      InstantCommand(
          [superstructure]() { superstructure->PositionHigh(); }),
      WaitCommand(0.9_s),
      DriveTrajectory(drive, &TrajectoryManager::GetInstance().GetTrajectory(
                                 allianceSidePrefix + "south-2cone_0_place1")),
      InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),
      WaitCommand(0.1_s),

      ParallelDeadlineGroup(
          DriveTrajectory(drive,
                          &TrajectoryManager::GetInstance().GetTrajectory(
                              allianceSidePrefix + "south-2cone_1_pick1")),
          SequentialCommandGroup(WaitCommand(0.25_s),
                                       InstantCommand([superstructure]() {
                                         superstructure->IntakeCone();
                                       }))),
      ParallelDeadlineGroup(
          DriveTrajectory(drive,
                          &TrajectoryManager::GetInstance().GetTrajectory(
                              allianceSidePrefix + "south-2cone_2_place3")),
          SequentialCommandGroup(WaitCommand(4.0_s),
                                       InstantCommand([superstructure]() {
                                         superstructure->PositionHigh();
                                       }))),
      InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); })
  );
}

frc::Pose2d South2Cone::GetStartingPose(bool isBlue) {
  static auto blueStartingPose = TrajectoryManager::GetInstance()
                                     .GetTrajectory("blue-south-2cone_0_place1")
                                     .GetInitialPose();
  static auto redStartingPose = TrajectoryManager::GetInstance()
                                    .GetTrajectory("red-south-2cone_0_place1")
                                    .GetInitialPose();
  if (isBlue)
    return blueStartingPose;
  else
    return redStartingPose;
}
