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
#include <units/length.h>
#include <wpi/json.h>

#include "opencv2/core/mat.hpp"
#include "photonlib/PhotonPoseEstimator.h"
#include "util/photonlib2/PhotonPoseEstimator.h"
#include "Constants.h"

using namespace frc;
using namespace photonlib;
using namespace wpi;
using namespace VisionConstants;

AprilTagFieldLayout CustomFieldLayout() {
  AprilTag tagTwo{2, Pose3d(0_m, 0_m, 0_m, Rotation3d())};
  AprilTag tagSix{6, Pose3d(8_in, 0_m, 0_m, Rotation3d())};
  return AprilTagFieldLayout({tagTwo, tagSix}, 10_m, 10_m);
}

Vision::Vision()
    : m_poseEstimator(/*LoadAprilTagLayoutField(AprilTagField::k2023ChargedUp)*/
                      CustomFieldLayout(),
                      m_cameraMatrix,
                      m_distortionCoefficients,
                      photonlib::PhotonCamera{"front"}, 
                      kRobotToCam),
    m_oldPoseEstimator(CustomFieldLayout(),
              photonlib::LOWEST_AMBIGUITY,
              photonlib::PhotonCamera("front"),
              kRobotToCam) {
  json j = m_poseEstimator.GetFieldLayout();
  SmartDashboard::PutString("AprilTags", j.dump());
}

void Vision::Periodic() {}

std::optional<photonlib2::EstimatedRobotPose> Vision::GetEstimatedGlobalPose(
    const frc::Pose3d& prevEstimatedRobotPose) {
  m_poseEstimator.SetReferencePose(prevEstimatedRobotPose);
  m_oldPoseEstimator.SetReferencePose(prevEstimatedRobotPose);
  auto updte = m_oldPoseEstimator.Update();
  SmartDashboard::PutNumber("oldpv/X", updte->estimatedPose.ToPose2d().X().value());
  SmartDashboard::PutNumber("oldpv/Y", updte->estimatedPose.ToPose2d().Y().value());
  SmartDashboard::PutNumber("oldpv/Theta", updte->estimatedPose.ToPose2d().Rotation().Radians().value());
  return m_poseEstimator.Update();
}
