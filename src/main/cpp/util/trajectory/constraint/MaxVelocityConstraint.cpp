// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/trajectory/constraint/MaxVelocityConstraint.h"

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

using namespace frc;

MaxVelocityConstraint::MaxVelocityConstraint(units::meters_per_second_t maxVelocityX,
                                             units::meters_per_second_t maxVelocityY,
                                             units::radians_per_second_t maxRotationalVelocity) :
    m_maxVelocityX{maxVelocityX},
    m_maxVelocityY{maxVelocityY},
    m_maxRotationalVelocity{maxRotationalVelocity} {}

double MaxVelocityConstraint::MaxVelocityNormForward(Pose2d currentPose,
                                                     Pose2d endPose,
                                                     ChassisSpeeds startVelocityHat,
                                                     double startVelocityNorm,
                                                     ChassisSpeeds endVelocityHat) {

}

double MaxVelocityConstraint::MaxVelocityNormBackward(Pose2d currentPose,
                                                      Pose2d endPose,
                                                      ChassisSpeeds startVelocityHat,
                                                      ChassisSpeeds endVelocityHat,
                                                      double endVelocityNorm) {
}
