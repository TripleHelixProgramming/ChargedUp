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
           // Left camera
            //       737.1375625617618,
            //   0.0, 605.0596973248307, 0.0, 737.332104946256,
            //   326.2280047911644, 0.0, 0.0, 1.0
           // Right camera
           742.483468466319,
       0.0, 637.2421086110966, 0.0, 741.833232408462, 401.5930628745256, 0.0,
       0.0, 1.0
       // Josh's XPS 13 webcam
       // 715.4570872534645, 0.0, 503.2741860156487, 0.0, 718.1743279814092,
       // 319.62675877827525, 0.0, 0.0, 1.0
      );
  cv::Mat m_distortionCoefficients =
      (cv::Mat_<double>(5, 1) <<
           // Left camera
            //       0.048988140593189386,
            //   0.1507107694319857, -0.0013521811213351104,
            //   -8.674674516252695E-4, -0.2012522991632792
           // Right camera
           0.10050662325551381,
       -0.048995749738143635, -0.0014568776758625078, 0.0012852294132110506,
       -0.15731800217755504
       // Josh's XPS 13 webcam
       // -0.1993374890518083, 1.056200497191576, 0.009578356882858418,
       // 0.008707188436370845, -1.5919105322821563
      );
  //  0.288184500781687, -1.03063206816648, -0.0044067255497647895,
  //  -0.0013635121873952262, 1.2849267796860973);

  photonlib2::PhotonPoseEstimator m_poseEstimator;
};
