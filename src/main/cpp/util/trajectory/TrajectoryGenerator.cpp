// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/trajectory/TrajectoryGenerator.h"

#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "util/trajectory/Trajectory.h"

using namespace frc;

Trajectory TrajectoryGenerator::Generate(Pose2d start, Pose2d end) {
  // Given C₂ continous functions x(t), y(t), θ(t) where t ∈ [0, 1]. Time is denoted τ.
  //
  //       [dx/dτ]
  //   v = [dy/dτ]
  //       [dθ/dτ]
  //
  //       [dx/dt]
  //   vₜ = [dy/dt]
  //       [dθ/dt]
  //
  // We state here that
  //
  //   v̂ = v̂ₜ         (1)
  // 
  // Given a discretized set of poses on the previously defined functions, we can infer
  // v̂ at all states from (1).
  //
  // The problem statement is maximize the unknowns |v| subject to the robot's constraints.
  // We solve the problem with dynamic programming
  //
  //        |
  //        |
  //        |               - - - - - - - - - - -              - - - - - - - - -
  //        |
  //   |v|  |                                     - - - - - -
  //        | - - - - - - - 
  //        |
  //        |
  //        |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
  //                                             t
  //
  // A forward pass is applied
  //
  //        |
  //        |
  //        |            / - - - - - - - - - - -           / - - - - - - - - - -
  //        |           /                                 /
  //   |v|  |          /                         - - - - /
  //        |   / - - / 
  //        |  /
  //        | /
  //        |/ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
  //                                             t
  //
  // A backward pass is applied
  //
  //        |
  //        |
  //        |            / - - - - - - - - - - \           / - - - - - \
  //        |           /                       \         /             \
  //   |v|  |          /                         \ - - - /               \
  //        |   / - - /                                                   \
  //        |  /                                                           \
  //        | /                                                             \
  //        |/ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ \ _
  //                                             t
  std::vector<Pose2d> pose;
  std::vector<ChassisSpeeds> v_hat;
  std::vector<double> v_norm;

  std::vector<Trajectory::State> trajectoryStates;
  for (size_t index = 0; index < pose.size(); ++index) {
    trajectoryStates.emplace_back(1, pose[index], ChassisSpeeds{v_norm[index] * v_hat[index].vx,
                                                                v_norm[index] * v_hat[index].vy,
                                                                v_norm[index] * v_hat[index].omega});
  }

  return Trajectory();
}