// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "subsystems/Vision.h"

#include <frc/StateSpaceUtil.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/RobotPoseEstimator.h>

#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <wpi/json.h>

#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <memory>
#include <vector>

using namespace frc;
using namespace photonlib;
using namespace wpi;

Vision::Vision()
    : m_aprilTagFieldLayout(LoadAprilTagLayoutField(AprilTagField::k2023ChargedUp)) {
  json j = m_aprilTagFieldLayout;
  SmartDashboard::PutString("AprilTags", j.dump());

//   frc::Transform3d robotToCam =
//       frc::Transform3d(frc::Translation3d(0.5_m, 0_m, 0.5_m),
//                       frc::Rotation3d(0_rad, 0_rad, 0_rad));

//   // ... Add other cameras here

//   // Assemble the list of cameras & mount locations
//   std::vector<
//       std::pair<std::shared_ptr<photonlib::PhotonCamera>, frc::Transform3d>>
//       cameras;
//   cameras.push_back(std::make_pair(cameraOne, robotToCam));

//   photonlib::RobotPoseEstimator estimator(
//       aprilTags, photonlib::CLOSEST_TO_REFERENCE_POSE, cameras);
}


void Vision::Periodic() {}

frc::Pose2d Vision::GetEstimatedGlobalPose(
    const frc::Pose2d& estimatedRobotPose) {
  auto randVec = frc::MakeWhiteNoiseVector(0.1, 0.1, 0.1);
  return frc::Pose2d{estimatedRobotPose.X() + units::meter_t{randVec(0)},
                     estimatedRobotPose.Y() + units::meter_t{randVec(1)},
                     estimatedRobotPose.Rotation() +
                         frc::Rotation2d{units::radian_t{randVec(2)}}};
}
