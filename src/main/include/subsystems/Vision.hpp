// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <optional>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <units/time.h>

#include "util/photonlib2/PhotonPoseEstimator.hpp"

class Vision : public frc2::SubsystemBase {
 public:
  Vision();

  void Periodic() override;

  std::optional<photonlib::EstimatedRobotPose> GetEstimatedGlobalPose(
      const frc::Pose3d& estimatedRobotPose);

 private:
  // Swiped from photonvision JSON config file
  cv::Mat m_cameraMatrix =
      (cv::Mat_<double>(3, 3) <<
           // Global shutter camera
           736.6865916947205, 0.0, 668.4278137814107, 0.0, 735.8136833112791, 381.763582584205, 0.0, 0.0, 1.0
       // Josh's XPS 13 webcam
       // 715.4570872534645, 0.0, 503.2741860156487, 0.0, 718.1743279814092,
       // 319.62675877827525, 0.0, 0.0, 1.0
      );
  cv::Mat m_distortionCoefficients =
      (cv::Mat_<double>(5, 1) <<
           // Global shutter camera
           0.03867607808694732, 0.51429292832619, 2.083487128640753E-4, -1.7555893251705346E-4, -1.672677874077145
       // Josh's XPS 13 webcam
       // -0.1993374890518083, 1.056200497191576, 0.009578356882858418,
       // 0.008707188436370845, -1.5919105322821563
      );
  //  0.288184500781687, -1.03063206816648, -0.0044067255497647895,
  //  -0.0013635121873952262, 1.2849267796860973);

  photonlib2::PhotonPoseEstimator m_poseEstimator;
};
