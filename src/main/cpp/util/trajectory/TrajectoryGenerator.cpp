// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/trajectory/TrajectoryGenerator.h"

#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "util/trajectory/Trajectory.h"

using namespace frc;
using namespace units;

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

  // Discretize path into a set of poses and v_hats, assume a linear path.
  // TODO: use minimum-snap splines to generate paths with non-zero initial velocity.
  int N = 100;
  for (int index = 0; index < N; ++index) {
    
  }

  // Forward pass

  // Credit to rafi#0159 for this idea for the acceleration kinematics.
  //
  // vₓ₁² = 2aₓΔx + vₓ₀²
  // vᵧ₁² = 2aᵧΔy + vᵧ₀²
  // ω₁²  = 2αΔθ  + ω₀²

  // Backward pass

  // Convert set of poses, v_hats, and v_norms into a time-parameterized trajectory.
  std::vector<Trajectory::State> trajectoryStates;
  trajectoryStates.emplace_back(0.0_s, pose[0], ChassisSpeeds{v_norm[0] * v_hat[0].vx,
                                                              v_norm[0] * v_hat[0].vy,
                                                              v_norm[0] * v_hat[0].omega});
  for (size_t index = 1; index < pose.size(); ++index) {
    double ds;
    double v;
    if (std::abs((pose[index].X() - pose[index-1].X()).value()) > 1e-6) {
      ds = (pose[index].X() - pose[index - 1].X()).value();
      v = v_norm[index - 1] * (v_hat[index - 1].vx + v_hat[index].vx).value() / 2.0;
    } else if (std::abs((pose[index].Y() - pose[index-1].Y()).value()) > 1e-6) {
      ds = (pose[index].Y() - pose[index - 1].Y()).value();
      v = v_norm[index - 1] * (v_hat[index - 1].vy + v_hat[index].vy).value() / 2.0;
    } else {
      ds = (pose[index].Rotation() - pose[index - 1].Rotation()).Radians().value();
      v = v_norm[index - 1] * (v_hat[index - 1].omega + v_hat[index].omega).value() / 2.0;
    }
    trajectoryStates.emplace_back(second_t{ds / v} + trajectoryStates[index - 1].t, pose[index], 
                                  ChassisSpeeds{v_norm[index] * v_hat[index].vx,
                                                v_norm[index] * v_hat[index].vy,
                                                v_norm[index] * v_hat[index].omega});
  }

  return Trajectory(trajectoryStates);
}