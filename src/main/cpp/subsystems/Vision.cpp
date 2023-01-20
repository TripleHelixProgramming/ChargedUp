// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Vision.h"

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
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <units/length.h>
#include <wpi/json.h>

#include "Constants.h"

using namespace frc;
using namespace photonlib;
using namespace wpi;
using namespace VisionConstants;
using enum photonlib::PoseStrategy;

Vision::Vision()
    : m_poseEstimator(LoadAprilTagLayoutField(AprilTagField::k2023ChargedUp),
                      CLOSEST_TO_REFERENCE_POSE,
                      photonlib::PhotonCamera{"photonvision"}, kRobotToCam) {
  json j = m_poseEstimator.GetFieldLayout();
  SmartDashboard::PutString("AprilTags", j.dump());
}

void Vision::Periodic() {}

std::optional<EstimatedRobotPose> Vision::GetEstimatedGlobalPose(
    const frc::Pose3d& prevEstimatedRobotPose) {
  m_poseEstimator.SetReferencePose(prevEstimatedRobotPose);
  return m_poseEstimator.Update();
}
