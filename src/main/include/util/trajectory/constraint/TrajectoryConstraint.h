// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

/**
 * An interface for defining physical constraints on generated trajectories.
 */
struct TrajectoryConstraint {
  TrajectoryConstraint() = default;

  TrajectoryConstraint(const TrajectoryConstraint&) = default;
  TrajectoryConstraint& operator=(const TrajectoryConstraint&) = default;

  TrajectoryConstraint(TrajectoryConstraint&&) = default;
  TrajectoryConstraint& operator=(TrajectoryConstraint&&) = default;

  virtual ~TrajectoryConstraint() = default;

  virtual double MaxVelocityNormForward(frc::Pose2d currentPose,
                                        frc::Pose2d endPose,
                                        frc::ChassisSpeeds startVelocityHat,
                                        double startVelocityNorm,
                                        frc::ChassisSpeeds endVelocityHat) = 0;

  virtual double MaxVelocityNormBackward(frc::Pose2d currentPose,
                                         frc::Pose2d endPose,
                                         frc::ChassisSpeeds startVelocityHat,
                                         frc::ChassisSpeeds endVelocityHat,
                                         double endVelocityNorm) = 0;
};
