// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/North3ConeLow.hpp"

#include <string>

#include <frc/geometry/Pose2d.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/DriveTrajectory.hpp"
#include "subsystems/Superstructure.hpp"
#include "util/TrajectoryManager.hpp"

using namespace frc2;

North3ConeLow::North3ConeLow(SwerveDrive* drive, Superstructure* superstructure,
                             bool isBlue) {
  std::string allianceSidePrefix =
      isBlue ? "blue-" : "red-";  // TODO until we have a blue one
  AddCommands(
      DriveTrajectory(
          drive, &TrajectoryManager::GetTrajectory(
                     allianceSidePrefix + "north-2cone-high-chgstat_0_place9")),
      ParallelDeadlineGroup(
          SequentialCommandGroup(
              DriveTrajectory(drive, &TrajectoryManager::GetTrajectory(
                                         allianceSidePrefix +
                                         "north-2cone-high-chgstat_1_pick4"))),
          SequentialCommandGroup(
            InstantCommand([superstructure]() {
              superstructure->SetIntakeWheelSpeed(-0.5);
            }),
            WaitCommand(1_s),
            InstantCommand([superstructure]() {
              superstructure->IntakeCube();
              superstructure->m_stealth = true;
            })),
          SequentialCommandGroup(
            WaitCommand(TrajectoryManager::GetTrajectory(
                            allianceSidePrefix + "north-2cone-high-chgstat_1_pick4")
                            .GetTotalTime() -
                        0.2_s),
            InstantCommand(
                [superstructure]() { superstructure->IntakeCone(); }),
            WaitCommand(0.1_s), InstantCommand([superstructure]() {
              superstructure->m_stealth = false;
            }))),
      ParallelDeadlineGroup(
          DriveTrajectory(drive,
                          &TrajectoryManager::GetTrajectory(
                              allianceSidePrefix + "north-3cone-low_2_place8")),
          SequentialCommandGroup(
              WaitCommand(TrajectoryManager::GetTrajectory(
                              allianceSidePrefix + "north-3cone-low_2_place8")
                              .GetTotalTime() -
                          0.2_s),
              InstantCommand(
                  [superstructure]() { superstructure->m_flipConeMode = true; }))),
      InstantCommand(
                  [superstructure]() { superstructure->Outtake(); }),
      WaitCommand(0.25_s),
      InstantCommand(
                  [superstructure]() { superstructure->m_flipConeMode = false; }),
      ParallelDeadlineGroup(
          SequentialCommandGroup(
              WaitCommand(0.1_s),
              DriveTrajectory(
                  drive, &TrajectoryManager::GetTrajectory(
                             allianceSidePrefix + "north-3cone-low_3_pick3"))),
          SequentialCommandGroup(WaitCommand(1_s),
                                 InstantCommand([superstructure]() {
                                   superstructure->IntakeCube();
                                   superstructure->m_stealth = true;
                                 })),
          SequentialCommandGroup(
              WaitCommand(TrajectoryManager::GetTrajectory(
                              allianceSidePrefix + "north-3cone-low_3_pick3")
                              .GetTotalTime() -
                          0.2_s),
              InstantCommand(
                  [superstructure]() { superstructure->IntakeCone(); }),
              WaitCommand(0.1_s), InstantCommand([superstructure]() {
                superstructure->m_stealth = false;
              }))),
      DriveTrajectory(
          drive, &TrajectoryManager::GetTrajectory(allianceSidePrefix +
                                                   "north-3cone-low_4_place7")),
      InstantCommand([superstructure]() { superstructure->Outtake(); }));
}

frc::Pose2d North3ConeLow::GetStartingPose(bool isBlue) {
  static auto blueStartingPose =
      TrajectoryManager::GetTrajectory("blue-north-2cone-high-chgstat_0_place9")
          .GetInitialPose();
  static auto redStartingPose =
      TrajectoryManager::GetTrajectory("red-north-2cone-high-chgstat_0_place9")
          .GetInitialPose();
  if (isBlue)
    return blueStartingPose;
  else
    return redStartingPose;
}
