// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/photonlib2/PhotonPoseEstimator.hpp"

#include <chrono>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <frc/Errors.h>
#include <frc/apriltag/AprilTag.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPipelineResult.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <wpi/SmallVector.h>

using namespace frc;
using namespace units;

namespace photonlib2 {

PhotonPoseEstimator::PhotonPoseEstimator(frc::AprilTagFieldLayout aprilTagLayout,
                                         cv::Mat cameraMatrix,
                                         cv::Mat distortionCoefficients,
                                         photonlib::PhotonCamera&& camera,
                                         frc::Transform3d robotToCamera)
    : m_aprilTagLayout(std::move(aprilTagLayout)),
      m_cameraMatrix(std::move(cameraMatrix)),
      m_distortionCoefficients(std::move(distortionCoefficients)),
      m_camera(std::move(camera)),
      m_robotToCamera(robotToCamera),
      m_referencePose(frc::Pose3d()) {
}

std::optional<photonlib::EstimatedRobotPose> PhotonPoseEstimator::Update() {
  auto result = m_camera.GetLatestResult();

  if (!result.HasTargets()) {
    return std::nullopt;
  }

  auto targets = result.GetTargets();

  // List of corners mapped from 3d space (meters) to the 2d camera screen
  // (pixels).
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;

  // Add all target corners to main list of corners
  for (auto target : targets) {
    int id = target.GetFiducialId();
    if (auto tagCorners = CalcTagCorners(id); tagCorners.has_value()) {
      auto targetCorners =
          target.GetDetectedCorners();
      for (size_t cornerIdx = 0; cornerIdx < 4; ++cornerIdx) {
        imagePoints.emplace_back(targetCorners[cornerIdx].first, targetCorners[cornerIdx].second);
        objectPoints.emplace_back((*tagCorners)[cornerIdx]);
      }
    }
  }

  SmartDashboard::PutNumber("Number of corners", imagePoints.size());
  if (imagePoints.empty()) {
    return std::nullopt;
  }

  // Use OpenCV ITERATIVE solver
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  auto begin = std::chrono::system_clock::now();

  cv::solvePnP(objectPoints, imagePoints, m_cameraMatrix,
               m_distortionCoefficients, rvec, tvec, cv::SOLVEPNP_ITERATIVE);

  Pose3d pose = ToPose3d(tvec, rvec);

  auto end = std::chrono::system_clock::now();

  SmartDashboard::PutNumber("SQPNP/Translation/X", pose.X().value());
  SmartDashboard::PutNumber("SQPNP/Translation/Y", pose.Y().value());
  SmartDashboard::PutNumber("SQPNP/Translation/Z", pose.Z().value());
  SmartDashboard::PutNumber("SQPNP/Rotation/Roll (X)",
                            pose.Rotation().X().convert<degree>().value());
  SmartDashboard::PutNumber("SQPNP/Rotation/Pitch (Y)",
                            pose.Rotation().Y().convert<degree>().value());
  SmartDashboard::PutNumber("SQPNP/Rotation/Yaw (Z)",
                            pose.Rotation().Z().convert<degree>().value());
  SmartDashboard::PutNumber(
      "SQPNP/Time",
      std::chrono::duration_cast<std::chrono::milliseconds>(end - begin)
          .count());

  return photonlib::EstimatedRobotPose(pose.TransformBy(m_robotToCamera.Inverse()),
                            result.GetTimestamp());
}

cv::Point3d PhotonPoseEstimator::ToPoint3d(const Translation3d& translation) {
  return cv::Point3d(-translation.Y().value(),
                     -translation.Z().value(),
                     +translation.X().value());
}

Pose3d PhotonPoseEstimator::ToPose3d(const cv::Mat& tvec, const cv::Mat& rvec) {

  cv::Mat R;
  cv::Rodrigues(rvec, R); // R is 3x3

  R = R.t();  // rotation of inverse
  cv::Mat tvecI = -R * tvec; // translation of inverse

  // cv::Mat T = cv::Mat::eye(4, 4, R.type()); // T is 4x4
  // T( cv::Range(0,3), cv::Range(0,3) ) = R * 1; // copies R into T
  // T( cv::Range(0,3), cv::Range(3,4) ) = tvecI * 1; // copies tvec into T

  // T is a 4x4 matrix with the pose of the camera in the object frame

  Vectord<3> tv;
  tv[0] = +tvecI.at<double>(2, 0);
  tv[1] = -tvecI.at<double>(0, 0);
  tv[2] = -tvecI.at<double>(1, 0);
  Vectord<3> rv;
  rv[0] = +rvec.at<double>(2, 0);
  rv[1] = -rvec.at<double>(0, 0);
  rv[2] = +rvec.at<double>(1, 0);

  return Pose3d(
    Translation3d(
      meter_t{tv[0]},
      meter_t{tv[1]},
      meter_t{tv[2]}),
    Rotation3d(
      // radian_t{rv[0]},
      // radian_t{rv[1]},
      // radian_t{rv[2]}
      rv, radian_t{rv.norm()}
    ));
}

cv::Point3d PhotonPoseEstimator::TagCornerToObjectPoint(
    meter_t cornerX,
    meter_t cornerY,
    frc::Pose3d tagPose) {
  Translation3d cornerTrans = tagPose.Translation()
      + Translation3d(0.0_m, cornerX, cornerY).RotateBy(tagPose.Rotation());
  return ToPoint3d(cornerTrans);
}

std::optional<std::array<cv::Point3d, 4>> PhotonPoseEstimator::CalcTagCorners(int tagID) {
  if (auto tagPose = m_aprilTagLayout.GetTagPose(tagID); tagPose.has_value()) {
    return std::array{
      TagCornerToObjectPoint(-3_in, -3_in, *tagPose),
      TagCornerToObjectPoint(+3_in, -3_in, *tagPose),
      TagCornerToObjectPoint(+3_in, +3_in, *tagPose),
      TagCornerToObjectPoint(-3_in, +3_in, *tagPose)
    };
  } else {
    return std::nullopt;
  }
}

}  // namespace photonlib2
