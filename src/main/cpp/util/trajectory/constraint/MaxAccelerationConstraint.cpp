// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/trajectory/constraint/MaxAccelerationConstraint.h"

#include <limits>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

using namespace frc;
using namespace units;

MaxAccelerationConstraint::MaxAccelerationConstraint(meters_per_second_squared maxAccelerationX,
                                                     meters_per_second_squared maxAccelerationY,
                                                     radians_per_second_squared maxRotationalAcceleration) :
    m_maxAccelerationX{maxAccelerationX},
    m_maxAccelerationY{maxAccelerationY},
    m_maxRotationalAcceleration{maxRotationalAcceleration} {}

double MaxAccelerationConstraint::MaxVelocityNormForward(Pose2d currentPose,
                                                         Pose2d endPose,
                                                         ChassisSpeeds startVelocityHat,
                                                         double startVelocityNorm,
                                                         ChassisSpeeds endVelocityHat) {
  double v_norm = std::numeric_limits<double>::infinity();
  if (endVelocityHat.vx.value() != 0.0) {
    v_norm = std::min(v_norm, (m_maxVelocityX / endVelocityHat.vx).value());
  }
  if (endVelocityHat.vy.value() != 0.0) {
    v_norm = std::min(v_norm, (m_maxVelocityY / endVelocityHat.vy).value());
  }
  if (endVelocityHat.omega.value() != 0.0) {
    v_norm = std::min(v_norm, (m_maxRotationalVelocity / endVelocityHat.omega).value());
  }
  return v_norm;
}

double MaxAccelerationConstraint::MaxVelocityNormBackward(Pose2d currentPose,
                                                          Pose2d endPose,
                                                          ChassisSpeeds startVelocityHat,
                                                          ChassisSpeeds endVelocityHat,
                                                          double endVelocityNorm) {
  double v_norm = std::numeric_limits<double>::infinity();
  if (startVelocityHat.vx.value() != 0.0) {
    v_norm = std::min(v_norm, (m_maxVelocityX / startVelocityHat.vx).value());
  }
  if (startVelocityHat.vy.value() != 0.0) {
    v_norm = std::min(v_norm, (m_maxVelocityY / startVelocityHat.vy).value());
  }
  if (startVelocityHat.omega.value() != 0.0) {
    v_norm = std::min(v_norm, (m_maxRotationalVelocity / startVelocityHat.omega).value());
  }
  return v_norm;
}
