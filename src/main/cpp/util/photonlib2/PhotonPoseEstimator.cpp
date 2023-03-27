// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/photonlib2/PhotonPoseEstimator.hpp"

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
#include <photonlib/PhotonPoseEstimator.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <wpi/SmallVector.h>

using namespace frc;
using namespace units;

namespace photonlib2 {

PhotonPoseEstimator::PhotonPoseEstimator(
    frc::AprilTagFieldLayout aprilTagLayout, const cv::Mat* cameraMatrix,
    const cv::Mat* distortionCoefficients, photonlib::PhotonCamera&& camera,
    const frc::Transform3d* robotToCamera)
    : m_aprilTagLayout(std::move(aprilTagLayout)),
      m_cameraMatrix(cameraMatrix),
      m_distortionCoefficients(distortionCoefficients),
      m_camera(std::move(camera)),
      m_robotToCamera(robotToCamera),
      m_referencePose(frc::Pose3d()) {
  SmartDashboard::PutData("Pose Est 1", &m_pose1Field);
  SmartDashboard::PutData("Pose Est 2", &m_pose2Field);
}

frc::AprilTagFieldLayout PhotonPoseEstimator::GetFieldLayout() const {
  return m_aprilTagLayout;
}

frc::Pose3d PhotonPoseEstimator::GetReferencePose() const {
  return m_referencePose;
}

void PhotonPoseEstimator::SetReferencePose(frc::Pose3d referencePose) {
  m_referencePose = referencePose;
}

photonlib::PhotonCamera& PhotonPoseEstimator::GetCamera() {
  return m_camera;
}

void PhotonPoseEstimator::SetCameraMatrix(const cv::Mat* newCamMatrix) {
  m_cameraMatrix = newCamMatrix;
}

void PhotonPoseEstimator::SetDistortionCoefficients(
    const cv::Mat* newDistCoeff) {
  m_distortionCoefficients = newDistCoeff;
}

void PhotonPoseEstimator::SetRobotToCamera(
    const frc::Transform3d* newRobotToCam) {
  m_robotToCamera = newRobotToCam;
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
      auto targetCorners = target.GetDetectedCorners();
      for (size_t cornerIdx = 0; cornerIdx < 4; ++cornerIdx) {
        imagePoints.emplace_back(targetCorners.at(cornerIdx).first,
                                 targetCorners.at(cornerIdx).second);
        objectPoints.emplace_back((*tagCorners).at(cornerIdx));
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
    cv::solvePnP(objectPoints, imagePoints, *m_cameraMatrix,
                 *m_distortionCoefficients, rvec, tvec, false,
                 cv::SOLVEPNP_ITERATIVE);
    pose1 = pose2 = ToPose3d(tvec, rvec);
  } else {
    return std::nullopt;
  }

  auto end = std::chrono::system_clock::now();

  m_pose1Field.SetRobotPose(pose1.ToPose2d());
  m_pose2Field.SetRobotPose(Pose2d(pose1.X(), pose1.Y(), pose1.Rotation().Z()));

  pose1 = pose1.TransformBy(m_robotToCamera->Inverse());

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

  return photonlib::EstimatedRobotPose(pose1, result.GetTimestamp());
}

cv::Point3d PhotonPoseEstimator::ToPoint3d(const Translation3d& translation) {
  return cv::Point3d(-translation.Y().value(), -translation.Z().value(),
                     +translation.X().value());
}

Pose3d PhotonPoseEstimator::ToPose3d(const cv::Mat& tvec, const cv::Mat& rvec) {
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
                Rotation3d(rv, radian_t{rv.norm()}));
}

cv::Point3d PhotonPoseEstimator::TagCornerToObjectPoint(meter_t cornerX,
                                                        meter_t cornerY,
                                                        frc::Pose3d tagPose) {
  Translation3d cornerTrans =
      tagPose.Translation() +
      Translation3d(0.0_m, cornerX, cornerY).RotateBy(tagPose.Rotation());
  return ToPoint3d(cornerTrans);
}

std::optional<std::array<cv::Point3d, 4>> PhotonPoseEstimator::CalcTagCorners(
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

}  // namespace photonlib2