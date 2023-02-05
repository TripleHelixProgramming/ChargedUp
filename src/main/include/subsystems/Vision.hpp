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
  cv::Mat m_cameraMatrix = (cv::Mat_<double>(3, 3) <<

                                735.0038676061282,
                            0.0, 666.4393853583332, 0.0, 734.0230911175827,
                            388.3489253256961, 0.0, 0.0, 1.0);
  //  527.5798454437585, 0.0, 311.5337325176784, 0.0, 527.4487505192909,
  //  244.79944837215717, 0.0, 0.0, 1.0);
  cv::Mat m_distortionCoefficients =
      (cv::Mat_<double>(5, 1) <<

           0.10407329380796507,
       -0.07442029959264967, -4.0143549811508955e-4, -5.79173576686874e-4,
       -0.10751011668818884);
  //  0.288184500781687, -1.03063206816648, -0.0044067255497647895,
  //  -0.0013635121873952262, 1.2849267796860973);

  photonlib2::PhotonPoseEstimator m_poseEstimator;
  photonlib::PhotonPoseEstimator m_oldPoseEstimator;
};
