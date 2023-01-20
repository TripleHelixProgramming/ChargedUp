// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>

#include <frc/apriltag/AprilTagFieldLayout.h>

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  void Periodic() override;

  frc::Pose2d GetEstimatedGlobalPose(const frc::Pose2d& estimatedRobotPose);

 private:
  photonlib::PhotonCamera m_camera{"photonvision"};

  frc::AprilTagFieldLayout m_aprilTagFieldLayout;


};
