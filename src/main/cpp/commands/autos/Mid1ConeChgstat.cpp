// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/Mid1ConeChgstat.hpp"

#include <string>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/DriveTrajectory.hpp"
#include "subsystems/Superstructure.hpp"
#include "util/Trajectory.hpp"
#include "util/TrajectoryManager.hpp"

Mid1ConeChgstat::Mid1ConeChgstat(SwerveDrive* drive,
                                 Superstructure* superstructure, bool isBlue) {
  std::string allianceSidePrefix = isBlue ? "blue-" : "red-";
  AddCommands(
      frc2::InstantCommand(
          [superstructure]() { superstructure->PositionHigh(); }),
      frc2::WaitCommand(1.25_s),
      DriveTrajectory(drive,
                      TrajectoryManager::GetInstance().GetTrajectory(
                          allianceSidePrefix + "mid-1cone-chgstat_0_place6")),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),

      frc2::ParallelDeadlineGroup(
          DriveTrajectory(
              drive,
              TrajectoryManager::GetInstance().GetTrajectory(
                  allianceSidePrefix + "mid-1cone-chgstat_1_chgstat"),
              false),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.5_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->IntakeCone();
                                       }))),
      frc2::RunCommand(
          [drive]() {
            drive->Drive(frc::ChassisSpeeds{0_mps, 0_mps, 0.01_rad_per_s});
          },
          {drive}));
}

frc::Pose2d Mid1ConeChgstat::GetStartingPose(bool isBlue) {
  static auto blueStartingPose =
      TrajectoryManager::GetInstance()
          .GetTrajectory("blue-mid-1cone-chgstat_0_place6")
          .GetInitialPose();
  static auto redStartingPose =
      TrajectoryManager::GetInstance()
          .GetTrajectory("red-mid-1cone-chgstat_0_place6")
          .GetInitialPose();
  if (isBlue)
    return blueStartingPose;
  else
    return redStartingPose;
}
