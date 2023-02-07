// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/trajectory/TrajectoryGenerator.h"

#include <cmath>
#include <limits>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "util/trajectory/Trajectory.h"
#include "util/trajectory/constraint/TrajectoryConstraint.h"

using namespace frc;
using namespace units;

TrajectoryGenerator::TrajectoryGenerator(std::vector<TrajectoryConstraint> constraints) : m_constraints{constraints} {};

Trajectory TrajectoryGenerator::Generate(Pose2d start, Pose2d end) {
  // Given C₂ continous functions x(t), y(t), θ(t) where t ∈ [0, 1]. Time is
  // denoted τ.
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
  // Given a discretized set of poses on the previously defined functions, we
  // can infer v̂ at all states from (1).
  //
  // The problem statement is maximize the unknowns |v| subject to the robot's
  // constraints. We solve the problem with dynamic programming
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
  std::vector<Pose2d> poses;
  std::vector<ChassisSpeeds> v_hats;
  std::vector<double> v_norms;

  // Discretize path into a set of poses and v_hats, assume a linear path.
  // TODO: use minimum-snap splines to generate paths with non-zero initial
  // velocity.
  int N = 100;
  meter_t delta_x = end.X() - start.X();
  meter_t delta_y = end.Y() - start.X();
  Rotation2d delta_theta = end.Rotation() - start.Rotation();
  double linearVnorm = std::hypot(delta_x.value(), delta_y.value(), delta_theta.Radians().value());
  ChassisSpeeds linearVHat{
      meters_per_second_t{delta_x.value() / linearVnorm},
      meters_per_second_t{delta_y.value() / linearVnorm},
      radians_per_second_t{delta_theta.Radians().value() / linearVnorm}};
  for (int index = 0; index < N; ++index) {
    double scale = (double)index / (N - 1);
    poses.emplace_back(
        delta_x * scale + start.X(),
        delta_y * scale + start.Y(),
        delta_theta * scale + start.Rotation());
    v_hats.push_back(linearVHat);
  }

  // Rotate velocities to be robot relative, this makes the generator math easier.
  for (int index = 0; index < N; ++index) {
    v_hats[index] = ChassisSpeeds::FromFieldRelativeSpeeds(v_hats[index].vx, 
                                                           v_hats[index].vy, 
                                                           v_hats[index].omega, 
                                                           poses[index].Rotation());
  }

  // Forward pass
  for (int index = 0; index < N - 1; ++index) {
    double maxVelocityNorm = std::numeric_limits<double>::infinity();
    for (auto& constraint : m_constraints) {
      maxVelocityNorm = std::min(maxVelocityNorm, constraint.MaxVelocityNormForward(poses[index], 
                                                                                    poses[index + 1], 
                                                                                    v_hats[index], 
                                                                                    v_norms[index], 
                                                                                    v_hats[index + 1]));
      v_norms[index + 1] = maxVelocityNorm;
    }
  }

  // Backward pass
  for (int index = N - 1; index >= 0; --index) {
    double maxVelocityNorm = v_norms[index];
    for (auto& constraint : m_constraints) {
      maxVelocityNorm = std::min(maxVelocityNorm, constraint.MaxVelocityNormBackward(poses[index], 
                                                                                     poses[index + 1], 
                                                                                     v_hats[index],  
                                                                                     v_hats[index + 1],
                                                                                     v_norms[index + 1]));
      v_norms[index] = maxVelocityNorm;
    }
  }

  // Convert set of poses, v_hats, and v_norms into a time-parameterized
  // trajectory.
  std::vector<Trajectory::State> trajectoryStates;
  trajectoryStates.emplace_back(0.0_s, poses[0], ChassisSpeeds{v_norms[0] * v_hats[0].vx, 
                                                               v_norms[0] * v_hats[0].vy,
                                                               v_norms[0] * v_hats[0].omega});
  for (size_t index = 1; index < poses.size(); ++index) {
    Twist2d delta = poses[index - 1].Log(poses[index]);
    second_t dt;
    if (delta.dx > 1e-6_m) {
      dt = delta.dx / ((v_norms[index - 1] * v_hats[index - 1].vx + v_norms[index] * v_hats[index].vx) / 2.0);
    } else if (delta.dy > 1e-6_m) {
      dt = delta.dy / ((v_norms[index - 1] * v_hats[index - 1].vx + v_norms[index] * v_hats[index].vx) / 2.0);
    } else {
      dt = delta.dtheta / ((v_norms[index - 1] * v_hats[index - 1].omega + v_norms[index] * v_hats[index].omega) / 2.0);
    }
    trajectoryStates.emplace_back(
        second_t{dt} + trajectoryStates[index - 1].t, poses[index],
        // Convert velocity back into field relative from robot relative.
        ChassisSpeeds::FromFieldRelativeSpeeds(v_norms[index] * v_hats[index].vx,
                                               v_norms[index] * v_hats[index].vy,
                                               v_norms[index] * v_hats[index].omega,
                                               -poses[index].Rotation()));
  }

  return Trajectory(trajectoryStates);
}
