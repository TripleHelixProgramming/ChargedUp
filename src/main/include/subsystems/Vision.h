// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <optional>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <units/time.h>

#include "util/photonlib2/PhotonPoseEstimator.h"

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  void Periodic() override;

  std::optional<photonlib2::EstimatedRobotPose> GetEstimatedGlobalPose(
      const frc::Pose3d& estimatedRobotPose);

 private:
  photonlib2::PhotonPoseEstimator m_poseEstimator;
};
