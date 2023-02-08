// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <units/acceleration.h>
#include <units/angular_acceleration.h>

#include "util/trajectory/constraint/TrajectoryConstraint.h"

class MaxAccelerationConstraint : public TrajectoryConstraint {
 public:
  explicit MaxAccelerationConstraint(double maxAccelerationX,
                                     double maxAccelerationY,
                                     double maxRotationalAcceleration);

  double MaxVelocityNormForward(frc::Pose2d currentPose, frc::Pose2d endPose,
                                frc::ChassisSpeeds startVelocityHat,
                                double startVelocityNorm,
                                frc::ChassisSpeeds endVelocityHat) override;

  double MaxVelocityNormBackward(frc::Pose2d currentPose, frc::Pose2d endPose,
                                 frc::ChassisSpeeds startVelocityHat,
                                 frc::ChassisSpeeds endVelocityHat,
                                 double endVelocityNorm) override;

 private:
  double m_maxAccelerationX;
  double m_maxAccelerationY;
  double m_maxRotationalAcceleration;
};
