// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/North2ConeChgstat.hpp"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/DriveTrajectory.hpp"
#include "subsystems/Superstructure.hpp"

North2ConeChgstat::North2ConeChgstat(SwerveDrive* drive,
                                     Superstructure* superstructure,
                                     const TrajectoryManager* trajManager)
    : m_drive(drive), m_trajManager(trajManager) {
  AddCommands(
      frc2::InstantCommand(
          [superstructure]() { superstructure->PositionHigh(); }),
      frc2::WaitCommand(0.9_s),
      DriveTrajectory(m_drive,
                      &m_trajManager->GetTrajectory("north-2cone-chgstat_0_place9")),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),
      frc2::WaitCommand(0.1_s),

      frc2::ParallelDeadlineGroup(
          DriveTrajectory(m_drive,
                          &m_trajManager->GetTrajectory("north-2cone-chgstat_1_pick4")),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.25_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->IntakeCone();
                                       }))),

      DriveTrajectory(m_drive, &m_trajManager->GetTrajectory(
                                   "north-2cone-chgstat_2_align7")),
      frc2::ParallelDeadlineGroup(
          DriveTrajectory(m_drive, &m_trajManager->GetTrajectory(
                                       "north-2cone-chgstat_3_place7")),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.1_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->PositionHigh();
                                       }))),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),

      frc2::ParallelDeadlineGroup(
          DriveTrajectory(
              m_drive,
              &m_trajManager->GetTrajectory("north-2cone-chgstat_4_chgstat"),
              false),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.25_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->IntakeCone();
                                       }))),
      frc2::RunCommand(
          [drive]() {
            drive->Drive(frc::ChassisSpeeds{0_mps, 0_mps, 0.01_rad_per_s});
          },
          {drive}));
}
