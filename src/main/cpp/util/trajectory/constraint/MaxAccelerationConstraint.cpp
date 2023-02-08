// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/trajectory/constraint/MaxAccelerationConstraint.h"

#include <limits>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/acceleration.h>

using namespace frc;
using namespace units;
using namespace units::literals;

MaxAccelerationConstraint::MaxAccelerationConstraint(double maxAccelerationX,
                                                     double maxAccelerationY,
                                                     double maxRotationalAcceleration) :
    m_maxAccelerationX{maxAccelerationX},
    m_maxAccelerationY{maxAccelerationY},
    m_maxRotationalAcceleration{maxRotationalAcceleration} {}

double MaxAccelerationConstraint::MaxVelocityNormForward(Pose2d currentPose,
                                                         Pose2d endPose,
                                                         ChassisSpeeds startVelocityHat,
                                                         double startVelocityNorm,
                                                         ChassisSpeeds endVelocityHat) {
  double v_norm = std::numeric_limits<double>::infinity();
  Twist2d delta = currentPose.Log(endPose);
  double dx = delta.dx.value();
  double dy = delta.dy.value();
  double dtheta = delta.dtheta.value();
  double vx_hat = endVelocityHat.vx.value();
  double vy_hat = endVelocityHat.vy.value();
  double omega_hat = endVelocityHat.omega.value();
  if (endVelocityHat.vx.value() != 0.0) {
    if (vx_hat > 0.0) {
      double v = std::sqrt(2.0 * dx * m_maxAccelerationX + std::pow(startVelocityNorm * startVelocityHat.vx.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / endVelocityHat.vx.value()));
    } else {
      double v = std::sqrt(2.0 * dx * -m_maxAccelerationX + std::pow(startVelocityNorm * startVelocityHat.vx.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / endVelocityHat.vx.value()));
    }
  }
  if (endVelocityHat.vy.value() != 0.0) {
    if (vy_hat > 0.0) {
      double v = std::sqrt(2.0 * dy * m_maxAccelerationY + std::pow(startVelocityNorm * startVelocityHat.vy.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / endVelocityHat.vy.value()));
    } else {
      double v = std::sqrt(2.0 * dy * -m_maxAccelerationY + std::pow(startVelocityNorm * startVelocityHat.vy.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / endVelocityHat.vy.value()));
    }
  }
  if (endVelocityHat.omega.value() != 0.0) {
    if (omega_hat > 0.0) {
      double v = std::sqrt(2.0 * dtheta * m_maxRotationalAcceleration + std::pow(startVelocityNorm * startVelocityHat.omega.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / endVelocityHat.omega.value()));
    } else {
      double v = std::sqrt(2.0 * dtheta * -m_maxRotationalAcceleration + std::pow(startVelocityNorm * startVelocityHat.omega.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / endVelocityHat.omega.value()));
    }
  }
  return v_norm;
}

double MaxAccelerationConstraint::MaxVelocityNormBackward(Pose2d currentPose,
                                                          Pose2d endPose,
                                                          ChassisSpeeds startVelocityHat,
                                                          ChassisSpeeds endVelocityHat,
                                                          double endVelocityNorm) {
  double v_norm = std::numeric_limits<double>::infinity();
  Twist2d delta = currentPose.Log(endPose);
  double dx = delta.dx.value();
  double dy = delta.dy.value();
  double dtheta = delta.dtheta.value();
  double vx_hat = startVelocityHat.vx.value();
  double vy_hat = startVelocityHat.vy.value();
  double omega_hat = startVelocityHat.omega.value();
  if (startVelocityHat.vx.value() != 0.0) {
    if (vx_hat > 0.0) {
      double v = std::sqrt(2.0 * dx * m_maxAccelerationX + std::pow(endVelocityNorm * endVelocityHat.vx.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / startVelocityHat.vx.value()));
    } else {
      double v = std::sqrt(2.0 * dx * -m_maxAccelerationX + std::pow(endVelocityNorm * endVelocityHat.vx.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / startVelocityHat.vx.value()));
    }
  }
  if (startVelocityHat.vy.value() != 0.0) {
    if (vy_hat > 0.0) {
      double v = std::sqrt(2.0 * dy * m_maxAccelerationY + std::pow(endVelocityNorm * endVelocityHat.vy.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / startVelocityHat.vy.value()));
    } else {
      double v = std::sqrt(2.0 * dy * -m_maxAccelerationY + std::pow(endVelocityNorm * endVelocityHat.vy.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / startVelocityHat.vy.value()));
    }
  }
  if (startVelocityHat.omega.value() != 0.0) {
    if (omega_hat > 0.0) {
      double v = std::sqrt(2.0 * dtheta * m_maxRotationalAcceleration + std::pow(endVelocityNorm * endVelocityHat.omega.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / startVelocityHat.omega.value()));
    } else {
      double v = std::sqrt(2.0 * dtheta * -m_maxRotationalAcceleration + std::pow(endVelocityNorm * endVelocityHat.omega.value(), 2));
      v_norm = std::min(v_norm, std::abs(v / startVelocityHat.omega.value()));
    }
  }
  return v_norm;
}
