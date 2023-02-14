// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "commands/autos/North2ConeCharge.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "commands/DriveTrajectory.hpp"
#include "subsystems/Superstructure.hpp"

North2ConeCharge::North2ConeCharge(SwerveDrive* drive,
                                   Superstructure* superstructure,
                                   const TrajectoryManager* trajManager)
    : m_drive(drive), m_trajManager(trajManager) {
  AddCommands(
      frc2::InstantCommand(
          [superstructure]() { superstructure->PositionConeHigh(); }),
      frc2::WaitCommand(1.25_s),
      DriveTrajectory(m_drive,
                      &m_trajManager->GetTrajectory("north_place_grid3x1")),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); }),


      frc2::ParallelDeadlineGroup(
          DriveTrajectory(m_drive,
                          &m_trajManager->GetTrajectory("north_pick4")),
          frc2::SequentialCommandGroup(frc2::WaitCommand(0.5_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->IntakeCone();
                                       }))
      ),
      frc2::ParallelDeadlineGroup(
          DriveTrajectory(m_drive,
                          &m_trajManager->GetTrajectory("north_place_grid3x3")),
          frc2::SequentialCommandGroup(frc2::WaitCommand(2.0_s),
                                       frc2::InstantCommand([superstructure]() {
                                         superstructure->PositionConeHigh();
                                       }))),
      frc2::WaitCommand(1.0_s),
      frc2::InstantCommand(
          [superstructure]() { superstructure->SetExtenderPosition(false); })
      // DriveTrajectory(m_drive,
      //                 &m_trajManager->GetTrajectory("north_charging_station")));
  );
}
