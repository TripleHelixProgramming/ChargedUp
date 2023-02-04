// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <array>
#include <map>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include "frc/geometry/Translation2d.h"
#include "frc/geometry/Translation3d.h"
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>

namespace photonlib2 {

class PhotonPoseEstimator {
 public:
  /**
   * Create a new PhotonPoseEstimator.
   *
   * @param aprilTagLayout A AprilTagFieldLayout linking AprilTag IDs to Pose3ds with
   * respect to the FIRST field.
   * @param cameraMatrix PhotonCameras and
   * @param robotToCamera Transform3d from the center of the robot to the camera
   * mount positions (ie, robot ➔ camera).
   */
  PhotonPoseEstimator(frc::AprilTagFieldLayout aprilTagLayout,
                      cv::Mat cameraMatrix,
                      cv::Mat distortionCoefficients,
                      photonlib::PhotonCamera&& camera,
                      frc::Transform3d robotToCamera);

  /**
   * Get the AprilTagFieldLayout being used by the PositionEstimator.
   *
   * @return the AprilTagFieldLayout
   */
  frc::AprilTagFieldLayout GetFieldLayout() const { return m_aprilTagLayout; }

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
  void SetReferencePose(frc::Pose3d referencePose) {
    m_referencePose = referencePose;
  }

  /**
   * Update the pose estimator. Internally grabs a new PhotonPipelineResult from
   * the camera and process it.
   */
  std::optional<photonlib::EstimatedRobotPose> Update();

  photonlib::PhotonCamera& GetCamera() { return m_camera; }

 private:
  frc::AprilTagFieldLayout m_aprilTagLayout;

  cv::Mat m_cameraMatrix;
  cv::Mat m_distortionCoefficients;

  photonlib::PhotonCamera m_camera;

  frc::Transform3d m_robotToCamera;

  frc::Pose3d m_referencePose;

  /**
   * Converts a Translation3d in the field coordinate system to an OpenCV Point3d
   * in OpenCV space.
   * 
   * @param translation the position in the field coordinate system
   * @return the position in OpenCV space
  */
  static inline cv::Point3d ToPoint3d(const frc::Translation3d& translation);

  static inline frc::Pose3d ToPose3d(const cv::Mat& tvec, const cv::Mat& rvec);

  /**
   * Transforms an AprilTag corner position (specified relative to the tag's
   * 2d coordinate system) to a point in OpenCV space.
   * 
   * In the tag's coordinate system, the center is the origin, +x is right,
   * and +y is up.
   * 
   * @param cornerX x-coordinate of AprilTag corner
   * @param cornerY y-coordinate of AprilTag corner
   * @param position position of AprilTag corner relative to the center of the
   *                 tag (origin center, x right, y up).
  */
  static inline cv::Point3d TagCornerToObjectPoint(units::meter_t cornerX,
                                            units::meter_t cornerY,
                                            frc::Pose3d tagPose);

  std::optional<std::array<cv::Point3d, 4>> CalcTagCorners(int tagID);

  // /**
  //  * Map from AprilTag ID in layout to positions of all four corners of the tag
  //  * in OpenCV 3d space.
  //  * //TODO check if ordering of points matters here
  // */
  // std::unordered_map<int, std::array<cv::Point3d, 4>> m_tagCornerObjectPoints;
};

}  // namespace photonlib2
