// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <units/angular_acceleration.h>
#include <units/acceleration.h>

#include "util/trajectory/constraint/TrajectoryConstraint.h"

class MaxAccelerationConstraint : public TrajectoryConstraint {
 public:
  MaxAccelerationConstraint(units::meters_per_second_squared maxAccelerationX,
                            units::meters_per_second_squared maxAccelerationY,
                            units::radians_per_second_squared maxRotationalAcceleration);

  double MaxVelocityNormForward(frc::Pose2d currentPose,
                                frc::Pose2d endPose,
                                frc::ChassisSpeeds startVelocityHat,
                                double startVelocityNorm,
                                frc::ChassisSpeeds endVelocityHat);

  double MaxVelocityNormBackward(frc::Pose2d currentPose,
                                 frc::Pose2d endPose,
                                 frc::ChassisSpeeds startVelocityHat,
                                 frc::ChassisSpeeds endVelocityHat,
                                 double endVelocityNorm);

 private:
  units::meters_per_second_squared m_maxAccelerationX;
  units::meters_per_second_squared m_maxAccelerationY;
  units::radians_per_second_squared m_maxRotationalAcceleration;
};
