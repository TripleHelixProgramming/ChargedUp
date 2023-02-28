// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <memory>
#include <optional>
#include <vector>

#include <frc/StateSpaceUtil.h>
#include <frc/Timer.h>
#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <opencv2/core/mat.hpp>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <units/length.h>
#include <wpi/json.h>

#include "Constants.hpp"
#include "util/photonlib2/PhotonPoseEstimator.hpp"

using namespace frc;
using namespace photonlib;
using namespace wpi;
using namespace VisionConstants;

AprilTagFieldLayout CustomFieldLayout() {
  AprilTag tagTwo{2, Pose3d(0_m, 0_m, 0_m, Rotation3d())};
  AprilTag tagSix{6, Pose3d(0_m, 0_m + 8_in, 0_m, Rotation3d())};
  return AprilTagFieldLayout({tagTwo, tagSix}, 10_m, 10_m);
}

// AprilTagFieldLayout CustomFieldLayout() {
//   AprilTag tagOne{1, Pose3d(0_m, 0_m, 0_m, Rotation3d())};
//   return AprilTagFieldLayout({tagOne}, 10_m, 10_m);
// }

Vision::Vision()
    : m_poseEstimator(LoadAprilTagLayoutField(AprilTagField::k2023ChargedUp),
                      // CustomFieldLayout(),
                      m_cameraMatrix, m_distortionCoefficients,
                      photonlib::PhotonCamera{"left"}, kRobotToLeftCam) {
  json j = m_poseEstimator.GetFieldLayout();
}

void Vision::Periodic() {}

std::optional<photonlib::EstimatedRobotPose> Vision::GetEstimatedGlobalPose(
    const frc::Pose3d& prevEstimatedRobotPose) {
  m_poseEstimator.SetReferencePose(prevEstimatedRobotPose);
  return m_poseEstimator.Update();
}
