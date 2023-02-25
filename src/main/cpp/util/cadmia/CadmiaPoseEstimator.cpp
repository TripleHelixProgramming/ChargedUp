// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/cadmia/CadmiaPoseEstimator.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
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

namespace cadmia {

CadmiaPoseEstimator::CadmiaPoseEstimator(
    frc::AprilTagFieldLayout aprilTagLayout, cv::Mat cameraMatrix,
    cv::Mat distortionCoefficients, cadmia::CadmiaCamera&& camera,
    frc::Transform3d robotToCamera)
    : m_aprilTagLayout(std::move(aprilTagLayout)),
      m_cameraMatrix(std::move(cameraMatrix)),
      m_distortionCoefficients(std::move(distortionCoefficients)),
      m_camera(std::move(camera)),
      m_robotToCamera(robotToCamera),
      m_referencePose(frc::Pose3d()) {
  SmartDashboard::PutData("Pose Est 1", &m_pose1Field);
  SmartDashboard::PutData("Pose Est 2", &m_pose2Field);
}

frc::AprilTagFieldLayout CadmiaPoseEstimator::GetFieldLayout() const {
  return m_aprilTagLayout;
}

frc::Pose3d CadmiaPoseEstimator::GetReferencePose() const {
  return m_referencePose;
}

void CadmiaPoseEstimator::SetReferencePose(frc::Pose3d referencePose) {
  m_referencePose = referencePose;
}

cadmia::CadmiaCamera& CadmiaPoseEstimator::GetCamera() {
  return m_camera;
}

std::optional<photonlib::EstimatedRobotPose> CadmiaPoseEstimator::Update() {
  auto result = m_camera.GetResult();
  auto targets = result.targets;

  // List of corners mapped from 3d space (meters) to the 2d camera screen
  // (pixels).
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;

  // Add all target corners to main list of corners
  for (auto target : targets) {
    int id = target.fiducialID;
    if (auto tagCorners = CalcTagCorners(id); tagCorners.has_value()) {
      auto targetCorners = target.corners;
      for (size_t cornerIdx = 0; cornerIdx < 4; ++cornerIdx) {
        imagePoints.emplace_back(targetCorners[cornerIdx].first,
                                 targetCorners[cornerIdx].second);
        objectPoints.emplace_back((*tagCorners)[cornerIdx]);
      }
    }
  }

  SmartDashboard::PutNumber("Number of corners", imagePoints.size());
  if (imagePoints.empty()) {
    return std::nullopt;
  }

  // Use OpenCV ITERATIVE solver
  // std::vector<cv::Mat> rvecs;
  // std::vector<cv::Mat> tvecs;

  Pose3d pose1;
  Pose3d pose2;

  auto begin = std::chrono::system_clock::now();

  if (objectPoints.size() >= 8) {
    cv::Mat rvec(3, 1, cv::DataType<double>::type);
    cv::Mat tvec(3, 1, cv::DataType<double>::type);
    cv::solvePnP(objectPoints, imagePoints, m_cameraMatrix,
                 m_distortionCoefficients, rvec, tvec, false,
                 cv::SOLVEPNP_ITERATIVE);
    pose1 = pose2 = ToPose3d(tvec, rvec);
  } else {
    return std::nullopt;
  }

  auto end = std::chrono::system_clock::now();

  m_pose1Field.SetRobotPose(pose1.ToPose2d());
  m_pose2Field.SetRobotPose(Pose2d(pose1.X(), pose1.Y(), pose1.Rotation().Z()));

  pose1 = pose1.TransformBy(m_robotToCamera.Inverse());

  SmartDashboard::PutNumber("SQPNP/Translation/X", pose1.X().value());
  SmartDashboard::PutNumber("SQPNP/Translation/Y", pose1.Y().value());
  SmartDashboard::PutNumber("SQPNP/Translation/Z", pose1.Z().value());
  SmartDashboard::PutNumber("SQPNP/Rotation/Roll (X)",
                            pose1.Rotation().X().convert<degree>().value());
  SmartDashboard::PutNumber("SQPNP/Rotation/Pitch (Y)",
                            pose1.Rotation().Y().convert<degree>().value());
  SmartDashboard::PutNumber("SQPNP/Rotation/Yaw (Z)",
                            pose1.Rotation().Z().convert<degree>().value());
  SmartDashboard::PutNumber(
      "SQPNP/Time",
      std::chrono::duration_cast<std::chrono::microseconds>(end - begin)
              .count() /
          1000.0);

  if (std::abs(pose1.Z().convert<cm>().value()) >= 20) {
    return std::nullopt;
  }

  return photonlib::EstimatedRobotPose(pose1, result.time);
}

cv::Point3d CadmiaPoseEstimator::ToPoint3d(const Translation3d& translation) {
  return cv::Point3d(-translation.Y().value(), -translation.Z().value(),
                     +translation.X().value());
}

Pose3d CadmiaPoseEstimator::ToPose3d(const cv::Mat& tvec, const cv::Mat& rvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);  // R is 3x3

  R = R.t();                  // rotation of inverse
  cv::Mat tvecI = -R * tvec;  // translation of inverse

  Vectord<3> tv;
  tv[0] = +tvecI.at<double>(2, 0);
  tv[1] = -tvecI.at<double>(0, 0);
  tv[2] = -tvecI.at<double>(1, 0);
  Vectord<3> rv;
  rv[0] = +rvec.at<double>(2, 0);
  rv[1] = -rvec.at<double>(0, 0);
  rv[2] = +rvec.at<double>(1, 0);

  return Pose3d(Translation3d(meter_t{tv[0]}, meter_t{tv[1]}, meter_t{tv[2]}),
                Rotation3d(
                    // radian_t{rv[0]},
                    // radian_t{rv[1]},
                    // radian_t{rv[2]}
                    rv, radian_t{rv.norm()}));
}

cv::Point3d CadmiaPoseEstimator::TagCornerToObjectPoint(meter_t cornerX,
                                                        meter_t cornerY,
                                                        frc::Pose3d tagPose) {
  Translation3d cornerTrans =
      tagPose.Translation() +
      Translation3d(0.0_m, cornerX, cornerY).RotateBy(tagPose.Rotation());
  return ToPoint3d(cornerTrans);
}

std::optional<std::array<cv::Point3d, 4>> CadmiaPoseEstimator::CalcTagCorners(
    int tagID) {
  if (auto tagPose = m_aprilTagLayout.GetTagPose(tagID); tagPose.has_value()) {
    return std::array{
        TagCornerToObjectPoint(-3_in, -3_in, *tagPose),
        TagCornerToObjectPoint(+3_in, -3_in, *tagPose),
        TagCornerToObjectPoint(+3_in, +3_in,
                               *tagPose),  // TODO change to match csys of
                                           // OpenCV (+y should be down)
        TagCornerToObjectPoint(-3_in, +3_in, *tagPose)};
  } else {
    return std::nullopt;
  }
}

}  // namespace cadmia