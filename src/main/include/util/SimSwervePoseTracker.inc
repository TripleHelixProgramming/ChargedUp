// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <cstddef>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <wpi/array.h>

#include "util/SimSwervePoseTracker.hpp"

template <size_t NumModules>
SimSwervePoseTracker<NumModules>::SimSwervePoseTracker(
    frc::SwerveDriveKinematics<NumModules> kinematics,
    const wpi::array<frc::SwerveModulePosition, NumModules>& modulePositions,
    const frc::Pose2d& initialPose)
    : m_kinematics(kinematics), m_pose(initialPose) {
  m_previousModulePositions = modulePositions;
  // for (size_t i = 0; i < NumModules; i++) {
  //   m_previousModulePositions[i] = {modulePositions[i].distance,
  //                                   modulePositions[i].angle};
  // }
}

template <size_t NumModules>
const frc::Pose2d& SimSwervePoseTracker<NumModules>::Update(
    const wpi::array<frc::SwerveModulePosition, NumModules>& modulePositions) {
  auto moduleDeltas =
      wpi::array<frc::SwerveModulePosition, NumModules>(wpi::empty_array);
  for (size_t index = 0; index < NumModules; index++) {
    auto lastPosition = m_previousModulePositions[index];
    auto currentPosition = modulePositions[index];
    moduleDeltas[index] = {currentPosition.distance - lastPosition.distance,
                           currentPosition.angle};

    m_previousModulePositions[index].distance = modulePositions[index].distance;
  }

  auto twist = m_kinematics.ToTwist2d(moduleDeltas);

  m_pose = m_pose.Exp(twist);

  return m_pose;
}
