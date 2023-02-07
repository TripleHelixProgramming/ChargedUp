// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

/**
 * An interface for defining physical constraints on generated trajectories.
 */
struct TrajectoryConstraint {
  virtual double MaxVelocityNormForward(frc::Pose2d currentPose,
                                        frc::Pose2d endPose,
                                        frc::ChassisSpeeds startVelocityHat,
                                        double startVelocityNorm,
                                        frc::ChassisSpeeds endVelocityHat);

  virtual double MaxVelocityNormBackward(frc::Pose2d currentPose,
                                         frc::Pose2d endPose,
                                         frc::ChassisSpeeds startVelocityHat,
                                         frc::ChassisSpeeds endVelocityHat,
                                         double endVelocityNorm);
};
