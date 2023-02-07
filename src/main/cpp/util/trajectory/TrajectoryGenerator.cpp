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

using namespace frc;
using namespace units;

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
};

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
  double linearVnorm =
      std::hypot((end.X() - start.X()).value(), (end.Y() - start.X()).value(),
                 (end.Rotation() - start.Rotation()).Radians().value());
  ChassisSpeeds linearVHat{
      meters_per_second_t{(end.X() - start.X()).value() / linearVnorm},
      meters_per_second_t{(end.Y() - start.Y()).value() / linearVnorm},
      radians_per_second_t{
          (end.Rotation() - start.Rotation()).Radians().value() / linearVnorm}};
  for (int index = 0; index < N; ++index) {
    double scale = (double)index / (N - 1);
    poses.emplace_back(
        (end.X() - start.X()) * scale + start.X(),
        (end.Y() - start.Y()) * scale + start.Y(),
        (end.Rotation() - start.Rotation()) * scale + start.Rotation());
    v_hats.push_back(linearVHat);
  }

  // Forward pass
  //
  //   vₓ₁² = 2aₓΔx + vₓ₀²
  //   vᵧ₁² = 2aᵧΔy + vᵧ₀²
  //   ω₁²  = 2αΔθ  + ω₀²
  for (int index = 0; index < N - 1; ++index) {
    double currentVelocityNorm = v_norms[index];
    ChassisSpeeds currentVelocityHat = v_hats[index];
    ChassisSpeeds nextVelocityHat = v_hats[index + 1];
    Twist2d delta = poses[index].Log(poses[index + 1]);
    auto maxAllowableSquaredVelocityX = meters_per_second_t{
        (2 * maxAccelerationX * delta.dx).value() +
        sgn(nextVelocityHat.vx.value()) *
            (currentVelocityNorm * currentVelocityHat.vx).value()};
    meters_per_second_t meters_per_second_t maxAllowableVelocityY = ;
    radians_per_second_t maxAllowableRotationalVelocity = ;

    double v_norm =
        std::min({maxAllowableVelocityX / v_hats[index].vx,
                  maxAllowableVelocityY / v_hats[index].vy,
                  maxAllowableRotationalVelocity / v_hats[index].omega});
  }

  // Backward pass
  for (int index = N - 1; index >= 0; --index) {
    double v_hat = std::numeric_limits<double>::infinity();
    Twist2d delta = poses[index].Log(poses[index + 1]);
    v_hat = std::min(v_hat, );
  }

  // Convert set of poses, v_hats, and v_norms into a time-parameterized
  // trajectory.
  std::vector<Trajectory::State> trajectoryStates;
  trajectoryStates.emplace_back(
      0.0_s, poses[0],
      ChassisSpeeds{v_norms[0] * v_hats[0].vx, v_norms[0] * v_hats[0].vy,
                    v_norms[0] * v_hats[0].omega});
  for (size_t index = 1; index < poses.size(); ++index) {
    double ds;
    double v;
    if (std::abs((poses[index].X() - poses[index - 1].X()).value()) > 1e-6) {
      ds = (poses[index].X() - poses[index - 1].X()).value();
      v = v_norms[index - 1] *
          (v_hats[index - 1].vx + v_hats[index].vx).value() / 2.0;
    } else if (std::abs((poses[index].Y() - poses[index - 1].Y()).value()) >
               1e-6) {
      ds = (poses[index].Y() - poses[index - 1].Y()).value();
      v = v_norms[index - 1] *
          (v_hats[index - 1].vy + v_hats[index].vy).value() / 2.0;
    } else {
      ds = (poses[index].Rotation() - poses[index - 1].Rotation())
               .Radians()
               .value();
      v = v_norms[index - 1] *
          (v_hats[index - 1].omega + v_hats[index].omega).value() / 2.0;
    }
    trajectoryStates.emplace_back(
        second_t{ds / v} + trajectoryStates[index - 1].t, poses[index],
        ChassisSpeeds{v_norms[index] * v_hats[index].vx,
                      v_norms[index] * v_hats[index].vy,
                      v_norms[index] * v_hats[index].omega});
  }

  return Trajectory(trajectoryStates);
}
