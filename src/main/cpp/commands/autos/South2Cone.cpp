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

South2Cone::South2Cone(SwerveDrive* drive, Superstructure* superstructure,
                       bool isBlue) {
  std::string allianceSidePrefix = isBlue ? "blue-" : "red-";
  AddCommands(
      frc2::InstantCommand(
          [superstructure]() { superstructure->PositionHigh(); }),
      frc2::WaitCommand(0.9_s),
      DriveTrajectory(drive, &TrajectoryManager::GetTrajectory(
                                 allianceSidePrefix + "south-2cone_0_place1")),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),
      frc2::WaitCommand(0.1_s),

      frc2::ParallelDeadlineGroup(
          DriveTrajectory(
              drive, &TrajectoryManager::GetTrajectory(allianceSidePrefix +
                                                       "south-2cone_1_pick1")),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.25_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->IntakeCone();
                                       }))),
      frc2::ParallelDeadlineGroup(
          DriveTrajectory(
              drive, &TrajectoryManager::GetTrajectory(allianceSidePrefix +
                                                       "south-2cone_2_place3")),
          frc2::SequentialCommandGroup(frc2::WaitCommand(4.0_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->PositionHigh();
                                       }))),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); })

      // frc2::ParallelDeadlineGroup(
      //     DriveTrajectory(drive,
      //                     &TrajectoryManager::GetTrajectory(allianceSidePrefix
      //                     + "north-2cone-chgstat_4_chgstat"), false),
      //     frc2::SequentialCommandGroup(frc2::WaitCommand(0.25_s),
      //                                  frc2::InstantCommand([superstructure]()
      //                                  {
      //                                    superstructure->IntakeCone();
      //                                  }))),
      // frc2::RunCommand(
      //     [drive]() { drive->Drive(frc::ChassisSpeeds{0_mps, 0_mps,
      //     0.01_rad_per_s}); }, {drive}
      // )
  );
}

frc::Pose2d South2Cone::GetStartingPose(bool isBlue) {
  static auto blueStartingPose =
      TrajectoryManager::GetTrajectory("blue-south-2cone_0_place1")
          .GetInitialPose();
  static auto redStartingPose =
      TrajectoryManager::GetTrajectory("red-south-2cone_0_place1")
          .GetInitialPose();
  if (isBlue)
    return blueStartingPose;
  else
    return redStartingPose;
}
