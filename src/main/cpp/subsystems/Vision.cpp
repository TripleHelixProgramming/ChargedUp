// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Vision.h"

#include <photonlib/PhotonCamera.h>
#include <frc/geometry/Pose2d.h>
#include <frc/StateSpaceUtil.h>

Vision::Vision() = default;

void Vision::Periodic() {}

frc::Pose2d Vision::GetEstimatedGlobalPose(
      const frc::Pose2d& estimatedRobotPose) {
  auto randVec = frc::MakeWhiteNoiseVector(0.1, 0.1, 0.1);
  return frc::Pose2d{estimatedRobotPose.X() + units::meter_t{randVec(0)},
                     estimatedRobotPose.Y() + units::meter_t{randVec(1)},
                     estimatedRobotPose.Rotation() +
                          frc::Rotation2d{units::radian_t{randVec(2)}}};
}