// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/trajectory/constraint/MaxVelocityConstraint.h"

#include <limits>
#include <iostream>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

using namespace frc;

MaxVelocityConstraint::MaxVelocityConstraint(
    units::meters_per_second_t maxVelocityX,
    units::meters_per_second_t maxVelocityY,
    units::radians_per_second_t maxRotationalVelocity)
    : m_maxVelocityX{maxVelocityX},
      m_maxVelocityY{maxVelocityY},
      m_maxRotationalVelocity{maxRotationalVelocity} {}

double MaxVelocityConstraint::MaxVelocityNormForward(
    Pose2d currentPose, Pose2d endPose, ChassisSpeeds startVelocityHat,
    double startVelocityNorm, ChassisSpeeds endVelocityHat) {
  double v_norm = std::numeric_limits<double>::infinity();
  if (endVelocityHat.vx.value() != 0.0) {
    v_norm = std::min(v_norm, (m_maxVelocityX / endVelocityHat.vx).value());
  }
  if (endVelocityHat.vy.value() != 0.0) {
    v_norm = std::min(v_norm, (m_maxVelocityY / endVelocityHat.vy).value());
  }
  if (endVelocityHat.omega.value() != 0.0) {
    v_norm = std::min(v_norm,
                      (m_maxRotationalVelocity / endVelocityHat.omega).value());
  }
  return v_norm;
}

double MaxVelocityConstraint::MaxVelocityNormBackward(
    Pose2d currentPose, Pose2d endPose, ChassisSpeeds startVelocityHat,
    ChassisSpeeds endVelocityHat, double endVelocityNorm) {
  std::cout << "Begin velocity backward" << std::endl;
  double v_norm = std::numeric_limits<double>::infinity();
  double vx = startVelocityHat.vx.value();
  double vy = startVelocityHat.vx.value();
  double omega = startVelocityHat.vx.value();
  if (vx != 0.0) {
    v_norm = std::min(v_norm, (m_maxVelocityX / vx).value());
  }
  if (vy != 0.0) {
    v_norm = std::min(v_norm, (m_maxVelocityY / vy).value());
  }
  if (omega != 0.0) {
    v_norm = std::min(v_norm, (m_maxRotationalVelocity / omega).value());
  }
  std::cout << "End velocity backward" << std::endl;
  return v_norm;
}
