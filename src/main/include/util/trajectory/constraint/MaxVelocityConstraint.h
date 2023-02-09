// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "util/trajectory/constraint/TrajectoryConstraint.h"

class MaxVelocityConstraint : public TrajectoryConstraint {
 public:
  explicit MaxVelocityConstraint(
      units::meters_per_second_t maxVelocityX,
      units::meters_per_second_t maxVelocityY,
      units::radians_per_second_t maxRotationalVelocity);

  MaxVelocityConstraint(const MaxVelocityConstraint&) = default;
  MaxVelocityConstraint& operator=(const MaxVelocityConstraint&) = default;

  MaxVelocityConstraint(MaxVelocityConstraint&&) = default;
  MaxVelocityConstraint& operator=(MaxVelocityConstraint&&) = default;

  double MaxVelocityNormForward(frc::Pose2d currentPose, frc::Pose2d endPose,
                                frc::ChassisSpeeds startVelocityHat,
                                double startVelocityNorm,
                                frc::ChassisSpeeds endVelocityHat) override;

  double MaxVelocityNormBackward(frc::Pose2d currentPose, frc::Pose2d endPose,
                                 frc::ChassisSpeeds startVelocityHat,
                                 frc::ChassisSpeeds endVelocityHat,
                                 double endVelocityNorm) override;

 private:
  units::meters_per_second_t m_maxVelocityX;
  units::meters_per_second_t m_maxVelocityY;
  units::radians_per_second_t m_maxRotationalVelocity;
};
