#pragma once

#include <cstddef>
#include "frc/kinematics/SwerveModulePosition.h"

#include <frc/kinematics/SwerveDriveKinematics.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <wpi/array.h>

/**
 * 
 */
template <size_t NumModules>
class SimSwervePoseTracker {
 public:
  /**
   * Constructs a SwerveDriveOdometry object.
   *
   * @param kinematics The swerve drive kinematics for your drivetrain.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param modulePositions The wheel positions reported by each module.
   * @param initialPose The starting position of the robot on the field.
   */
  SimSwervePoseTracker(
      frc::SwerveDriveKinematics<NumModules> kinematics,
      const wpi::array<frc::SwerveModulePosition, NumModules>& modulePositions = {
        frc::SwerveModulePosition{},
        frc::SwerveModulePosition{},
        frc::SwerveModulePosition{},
        frc::SwerveModulePosition{}
      },
      const frc::Pose2d& initialPose = frc::Pose2d{});

  /**
   * Returns the position of the robot on the field.
   * @return The pose of the robot.
   */
  const frc::Pose2d& GetPose() const { return m_pose; }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose over time. This also takes in an angle parameter
   * which is used instead of the angular rate that is calculated from forward
   * kinematics.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param modulePositions The current position of all swerve modules. Please
   * provide the positions in the same order in which you instantiated your
   * SwerveDriveKinematics.
   *
   * @return The new pose of the robot.
   */
  const frc::Pose2d& Update(
      const wpi::array<frc::SwerveModulePosition, NumModules>& modulePositions);

 private:
  frc::SwerveDriveKinematics<NumModules> m_kinematics;
  frc::Pose2d m_pose;

  wpi::array<frc::SwerveModulePosition, NumModules> m_previousModulePositions{
      wpi::empty_array};
};

#include "util/SimSwervePoseTracker.inc"
