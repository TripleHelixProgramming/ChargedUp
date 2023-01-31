#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

#include "opencv2/core/mat.hpp"
#include "photonlib/PhotonCamera.h"

namespace photonlib2 {

struct EstimatedRobotPose {
  /** The estimated pose */
  frc::Pose3d estimatedPose;
  /** The estimated time the frame used to derive the robot pose was taken, in
   * the same timebase as the RoboRIO FPGA Timestamp */
  units::second_t timestamp;

  EstimatedRobotPose(frc::Pose3d pose_, units::second_t time_)
      : estimatedPose(pose_), timestamp(time_) {}
};

class PhotonPoseEstimator {
 public:
  using map_value_type =
      std::pair<std::shared_ptr<photonlib::PhotonCamera>, frc::Transform3d>;
  using size_type = std::vector<map_value_type>::size_type;

  /**
   * Create a new PhotonPoseEstimator.
   *
   * @param aprilTags A AprilTagFieldLayout linking AprilTag IDs to Pose3ds with
   * respect to the FIRST field.
   * @param camera PhotonCameras and
   * @param robotToCamera Transform3d from the center of the robot to the camera
   * mount positions (ie, robot ➔ camera).
   */
  explicit PhotonPoseEstimator(frc::AprilTagFieldLayout aprilTags,
                               cv::Mat cameraMatrix, 
                               cv::Mat distortionCoefficients,
                               photonlib::PhotonCamera&& camera,
                               frc::Transform3d robotToCamera);

  /**
   * Get the AprilTagFieldLayout being used by the PositionEstimator.
   *
   * @return the AprilTagFieldLayout
   */
  frc::AprilTagFieldLayout GetFieldLayout() const { return m_aprilTags; }

  /**
   * Return the reference position that is being used by the estimator.
   *
   * @return the referencePose
   */
  frc::Pose3d GetReferencePose() const { return m_referencePose; }

  /**
   * Update the stored reference pose for use when using the
   * CLOSEST_TO_REFERENCE_POSE strategy.
   *
   * @param referencePose the referencePose to set
   */
  inline void SetReferencePose(frc::Pose3d referencePose) {
    this->m_referencePose = referencePose;
  }

  /**
   * Update the pose estimator. Internally grabs a new PhotonPipelineResult from
   * the camera and process it.
   */
  std::optional<EstimatedRobotPose> Update();

  inline photonlib::PhotonCamera& GetCamera() { return m_camera; }

 private:
  frc::AprilTagFieldLayout m_aprilTags;

  cv::Mat m_cameraMatrix;
  cv::Mat m_distortionCoefficients;

  photonlib::PhotonCamera m_camera;

  frc::Transform3d m_robotToCamera;

  frc::Pose3d m_referencePose;
};

}  // namespace photonlib2