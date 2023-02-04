// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/photonlib2/PhotonPoseEstimator.hpp"

#include <chrono>
#include <iostream>
#include <limits>
#include <map>
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
#include <photonlib/PhotonTrackedTarget.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <wpi/SmallVector.h>

using namespace frc;
using namespace units;

namespace photonlib2 {

PhotonPoseEstimator::PhotonPoseEstimator(frc::AprilTagFieldLayout tags,
                                         cv::Mat cameraMatrix,
                                         cv::Mat distortionCoefficients,
                                         photonlib::PhotonCamera&& cam,
                                         frc::Transform3d robotToCamera)
    : m_aprilTags(tags),
      m_cameraMatrix(cameraMatrix),
      m_distortionCoefficients(distortionCoefficients),
      m_camera(std::move(cam)),
      m_robotToCamera(robotToCamera),
      m_referencePose(frc::Pose3d()) {}

std::optional<EstimatedRobotPose> PhotonPoseEstimator::Update() {
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
  constexpr auto tagCornerToObjectPoint = [](Translation3d corner,
                                             Pose3d tagPose) {
    Translation3d point =
        corner.RotateBy(tagPose.Rotation()) + tagPose.Translation();
    return cv::Point3f(point.X().value(), point.Y().value(), point.Z().value());
  };

  for (auto target : targets) {
    int id = target.GetFiducialId();
    std::optional<Pose3d> tagPose = m_aprilTags.GetTagPose(id);
    if (tagPose) {
      std::vector<std::pair<double, double>> targetCorners =
          target.GetDetectedCorners();
      for (auto& corner : targetCorners) {
        imagePoints.emplace_back(corner.first, corner.second);
      }
      objectPoints.push_back(tagCornerToObjectPoint(
          Translation3d(-3_in, +3_in, 0_m), tagPose.value()));
      objectPoints.push_back(tagCornerToObjectPoint(
          Translation3d(+3_in, +3_in, 0_m), tagPose.value()));
      objectPoints.push_back(tagCornerToObjectPoint(
          Translation3d(+3_in, -3_in, 0_m), tagPose.value()));
      objectPoints.push_back(tagCornerToObjectPoint(
          Translation3d(-3_in, -3_in, 0_m), tagPose.value()));
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

  Vectord<3> rotationVector;
  rotationVector[0] = rvec.at<double>(2, 0);
  rotationVector[1] = rvec.at<double>(0, 0);
  rotationVector[2] = rvec.at<double>(1, 0);

  Pose3d pose{Translation3d(meter_t{tvec.at<double>(2, 0)},
                            meter_t{-tvec.at<double>(0, 0)},
                            meter_t{tvec.at<double>(1, 0)}),
              Rotation3d(rotationVector, radian_t{rotationVector.norm()})};

  auto end = std::chrono::system_clock::now();

  SmartDashboard::PutNumber("SQPNP/Translation/X", pose.X().value());
  SmartDashboard::PutNumber("SQPNP/Translation/Y", pose.Y().value());
  SmartDashboard::PutNumber("SQPNP/Translation/Z", pose.Z().value());
  SmartDashboard::PutNumber("SQPNP/Rotation/Yaw",
                            pose.Rotation().Z().convert<degree>().value());
  SmartDashboard::PutNumber("SQPNP/Rotation/Pitch",
                            pose.Rotation().Y().convert<degree>().value());
  SmartDashboard::PutNumber("SQPNP/Rotation/Roll",
                            pose.Rotation().X().convert<degree>().value());
  SmartDashboard::PutNumber(
      "SQPNP/Time",
      std::chrono::duration_cast<std::chrono::milliseconds>(end - begin)
          .count());

  return EstimatedRobotPose(pose.TransformBy(m_robotToCamera.Inverse()),
                            result.GetTimestamp());
}

}  // namespace photonlib2
