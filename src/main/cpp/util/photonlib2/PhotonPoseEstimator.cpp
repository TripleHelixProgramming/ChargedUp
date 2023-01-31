#include "util/photonlib2/PhotonPoseEstimator.h"

#include <iostream>
#include <limits>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>
#include <frc/geometry/Translation3d.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include "frc/apriltag/AprilTag.h"
#include "units/angle.h"
#include <wpi/SmallVector.h>

#include <frc/Errors.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <units/time.h>
#include <units/length.h>

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPipelineResult.h>
#include <photonlib/PhotonTrackedTarget.h>

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

  // List of corners mapped from 3d space (meters) to the 2d camera screen (pixels).
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;
  
  // Add all target corners to main list of corners
  auto translationToPoint3d = [](Translation3d corner, Pose3d tagPose) {
    Translation3d point = corner.RotateBy(tagPose.Rotation()) + tagPose.Translation();
    return cv::Point3f(point.X().value(), point.Y().value(), point.Z().value());
  };

  for (auto& target : targets) {
    int id = target.GetFiducialId();
    std::optional<Pose3d> tagPose = m_aprilTags.GetTagPose(id);
    if (tagPose) {
      wpi::SmallVector<std::pair<double, double>, 4> targetCorners = target.GetMinAreaRectCorners();
      for (auto& corner : targetCorners) {
        imagePoints.emplace_back(corner.first, corner.second);
      }
      objectPoints.push_back(translationToPoint3d(Translation3d(1_m, -1_m, 0_m), tagPose.value()));
      objectPoints.push_back(translationToPoint3d(Translation3d(1_m, 1_m, 0_m), tagPose.value()));
      objectPoints.push_back(translationToPoint3d(Translation3d(-1_m, 1_m, 0_m), tagPose.value()));
      objectPoints.push_back(translationToPoint3d(Translation3d(-1_m, -1_m, 0_m), tagPose.value()));
    }
  }

  if (imagePoints.empty()) {
    return std::nullopt;
  }

  // Use SQPnP
  cv::Mat rvec(3, 1, cv::DataType<double>::type);
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  cv::solvePnP(objectPoints, 
               imagePoints, 
               m_cameraMatrix, 
               m_distortionCoefficients, 
               rvec, 
               tvec, 
               cv::SOLVEPNP_SQPNP);

  return EstimatedRobotPose(Pose3d(Translation3d(meter_t{tvec.at<double>(0, 0)},
                                                 meter_t{tvec.at<double>(1, 0)},
                                                 meter_t{tvec.at<double>(2, 0)}), 
                                   Rotation3d(radian_t{rvec.at<double>(0, 0)},
                                              radian_t{rvec.at<double>(1, 0)},
                                              radian_t{rvec.at<double>(2, 0)})), 
                            result.GetTimestamp());
}

}  // namespace photonlib2