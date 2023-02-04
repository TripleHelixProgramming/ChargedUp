// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/trajectory/TrajectoryGenerator.h"

#include "util/trajectory/Trajectory.h"

using namespace frc;

Trajectory TrajectoryGenerator::Generate(Pose2d start, Pose2d end) {
  // Given variables x₀, y₀, θ₀, ẋ₀, ẏ₀, θ̇₀, final variables x₁, y₁, θ₁, and equations of motion
  //
  //   x₁ = x₀ + ẋ₀ * dt + 0.5 * ẍ₀ * dt²
  //   y₁ = x₀ + ẏ₀ * dt + 0.5 * ÿ₀ * dt²
  //   θ₁ = x₀ + θ̇₀ * dt + 0.5 * θ̈₀ * dt²
  //
  // Minimize dt subject to constraints
  //
  //   ẍ < 
  //   ÿ < 
  //   θ̈ < 


  // Minimize
  //
  //   dt
  //
  // Subject to
  //
  //   (x₁ - x₀) / dt = 0.5 * (ẋ₀ + ẋ₁)
  //   (y₁ - y₀) / dt = 0.5 * (ẏ₀ + ẏ₁)
  //   (θ₁ - θ₀) / dt = 0.5 * (θ̇₀ + θ̇₁)
  //
  // Reformulate problem with the substitution
  //
  //   h = 1 / dt
  //
  // Minimize
  //
  //   -h
  //
  // Subject to
  //
  //   (x₁ - x₀) / dt = 0.5 * (ẋ₀ + ẋ₁)
  //   (y₁ - y₀) / dt = 0.5 * (ẏ₀ + ẏ₁)
  //   (θ₁ - θ₀) / dt = 0.5 * (θ̇₀ + θ̇₁)
  //
  // 

  return Trajectory();
}