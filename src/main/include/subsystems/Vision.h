// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <optional>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <units/time.h>

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  void Periodic() override;

  std::optional<photonlib::EstimatedRobotPose> GetEstimatedGlobalPose(
      const frc::Pose3d& estimatedRobotPose);

 private:
  photonlib::PhotonPoseEstimator m_poseEstimator;
};
