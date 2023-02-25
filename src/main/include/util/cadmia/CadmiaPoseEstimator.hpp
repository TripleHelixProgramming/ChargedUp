// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <array>
#include <map>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/smartdashboard/Field2d.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include "util/cadmia/CadmiaCamera.hpp"
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>

namespace cadmia {

/**
 * Triple Helix's custom multi-tag PNP pose estimator. This is close to a
 * drop-in replacement for photonlib's pose estimator.
 */
class CadmiaPoseEstimator {
 public:
  /**
   * Create a new CadmiaPoseEstimator. Since PV doesn't publish camera
   * intrinsics to NetworkTables, they must be specified here.
   *
   * @param aprilTagLayout an AprilTagFieldLayout linking AprilTag IDs to
   * Pose3ds with respect to the FIRST field.
   * @param cameraMatrix a 3x3 matrix specific to the camera hardware, from
   * calibration
   * @param distortionCoefficients a 5x1 matrix also specific to camera
   * hardware, from calibration
   * @param camera the camera being used for pose estimation
   * @param robotToCamera transform from the center of the robot to the camera
   * mount positions (ie, robot -> camera)
   */
  CadmiaPoseEstimator(frc::AprilTagFieldLayout aprilTagLayout,
                      cv::Mat cameraMatrix, cv::Mat distortionCoefficients,
                      cadmia::CadmiaCamera&& camera,
                      frc::Transform3d robotToCamera);

  /**
   * Get the AprilTagFieldLayout
   *
   * @return the AprilTagFieldLayout
   */
  frc::AprilTagFieldLayout GetFieldLayout() const;

  /**
   * Return the reference position that is being used by the estimator.
   *
   * @return the referencePose
   */
  frc::Pose3d GetReferencePose() const;

  /**
   * Update the stored reference pose.
   *
   * @param referencePose the referencePose to set
   */
  void SetReferencePose(frc::Pose3d referencePose);

  /**
   * Get the camera used for pose estimation.
   *
   * @return a reference to the camera
   */
  cadmia::CadmiaCamera& GetCamera();

  /**
   * Update the pose estimator. Internally grabs a new PhotonPipelineResult from
   * the camera and process it.
   */
  std::optional<photonlib::EstimatedRobotPose> Update();

 private:
  /**
   * an AprilTagFieldLayout linking AprilTag IDs to Pose3ds with
   * respect to the FIRST field.
   */
  frc::AprilTagFieldLayout m_aprilTagLayout;

  /**
   * a 3x3 matrix specific to the camera hardware, from calibration
   */
  cv::Mat m_cameraMatrix;
  /**
   * a 5x1 matrix also specific to camera hardware, from calibration
   */
  cv::Mat m_distortionCoefficients;

  /**
   * the camera being used for pose estimation
   */
  cadmia::CadmiaCamera m_camera;

  /**
   * transform from the center of the robot to the camera
   * mount positions (ie, robot -> camera)
   */
  frc::Transform3d m_robotToCamera;

  /**
   * a previously calculated reference pose to used to check ambiguous poses
   */
  frc::Pose3d m_referencePose;

  frc::Field2d m_pose1Field;
  frc::Field2d m_pose2Field;

  /**
   * Converts a Translation3d in the field coordinate system to an OpenCV
   * Point3d in OpenCV space.
   *
   * @param translation the position in the field coordinate system
   * @return the position in OpenCV space
   */
  static inline cv::Point3d ToPoint3d(const frc::Translation3d& translation);

  /**
   * Converts solvePNP's generated tvec and rvec to a pose in the field
   * coordinate system.
   *
   * @param tvec the translation vector from the camera's coordinate frame to
   * the field coordinate system
   * @param rvec the rotation vector of the field coordinate system relative to
   * the camera's coordinate frame
   * @return the pose of the camera relative to the field coordinate system
   */
  static inline frc::Pose3d ToPose3d(const cv::Mat& tvec, const cv::Mat& rvec);

  /**
   * Transforms an AprilTag corner position (specified relative to the tag's
   * 2d coordinate system) to a point in OpenCV space.
   *
   * In the tag's coordinate system, if facing the tag, the center of it is the
   * origin, +x is right, and +y is up.
   *
   * @param cornerX x-coordinate of AprilTag corner
   * @param cornerY y-coordinate of AprilTag corner
   * @param position position of AprilTag corner relative to the center of the
   *                 tag (origin center, x right, y up).
   */
  static inline cv::Point3d TagCornerToObjectPoint(units::meter_t cornerX,
                                                   units::meter_t cornerY,
                                                   frc::Pose3d tagPose);

  /**
   * Using the AprilTag field layout, calculate the positions of all four
   * corners of the specified AprilTag in OpenCV world space. If the specified
   * AprilTag ID is not in the field layout, return nullopt.
   *
   * @param tagID the ID of the AprilTag
   * @return an array of the corner positions, or nullopt if not in the layout
   */
  std::optional<std::array<cv::Point3d, 4>> CalcTagCorners(int tagID);
};

}  // namespace cadmia
