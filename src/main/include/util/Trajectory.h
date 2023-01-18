// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <vector>

#include <Eigen/Core>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/json.h>

class Trajectory {
 public:
  struct State {
    units::second_t t;
    frc::Pose2d pose;
    units::meters_per_second_t vx;
    units::meters_per_second_t vy;
    units::radians_per_second_t omega;

    State Interpolate(const State& other, units::second_t newT) const;
  };

  Trajectory() = default;

  explicit Trajectory(std::vector<State> states);

  State Sample(units::second_t t) const;

  frc::Pose2d GetInitialPose() const;

  units::second_t GetTotalTime() const;

  std::vector<State> m_states;
};

void to_json(wpi::json& j, const Trajectory::State& state);
void from_json(const wpi::json& j, Trajectory::State& state);
void to_json(wpi::json& j, const Trajectory& traj);
void from_json(const wpi::json& j, Trajectory& traj);
