// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/Trajectory.hpp"

#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/base.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/json.h>

using namespace units;
using namespace frc;
using wpi::json;

Trajectory::State Trajectory::State::Interpolate(const Trajectory::State& other,
                                                 second_t newT) const {
  double scale = ((newT - t) / (other.t - t)).value();
  return Trajectory::State{newT, pose.Exp(pose.Log(other.pose) * scale),
                           (other.vx - vx) * scale + vx,
                           (other.vy - vy) * scale + vy,
                           (other.omega - omega) * scale + omega};
  return other;
}

Trajectory::Trajectory(std::vector<Trajectory::State> states)
    : m_states{states} {}

Trajectory::State Trajectory::Sample(second_t t) const {
  if (t < m_states[0].t) {
    return m_states[0];
  }
  if (t > GetTotalTime()) {
    return m_states.back();
  }

  // binary search to find two samples to lerp
  int low = 0;
  int high = m_states.size() - 1;

  while (low != high) {
    int mid = (low + high) / 2;
    if (m_states[mid].t < t) {
      low = mid + 1;
    } else {
      high = mid;
    }
  }

  auto previousState = m_states[low - 1];
  auto currentState = m_states[low];

  if ((currentState.t - previousState.t).value() == 0) {
    return currentState;
  }

  return previousState.Interpolate(currentState, t);
}

Pose2d Trajectory::GetInitialPose() const {
  return m_states[0].pose;
}

second_t Trajectory::GetTotalTime() const {
  return m_states.back().t;
}

void from_json(const json& j, Trajectory::State& state) {
  state.t = second_t{j.at("timestamp").get<double>()};
  state.pose = Pose2d(Translation2d(meter_t{j.at("x").get<double>()},
                                    meter_t{j.at("y").get<double>()}),
                      Rotation2d(radian_t{j.at("heading").get<double>()}));
  state.vx = meters_per_second_t{j.at("velocityX").get<double>()};
  state.vy = meters_per_second_t{j.at("velocityY").get<double>()};
  state.omega = radians_per_second_t{j.at("angularVelocity").get<double>()};
}

void to_json(json& j, const Trajectory::State& state) {
  j = json{{"timestamp", state.t.value()},
           {"x", state.pose.X().value()},
           {"y", state.pose.Y().value()},
           {"heading", state.pose.Rotation().Radians().value()},
           {"velocityX", state.vx.value()},
           {"velocityY", state.vy.value()},
           {"angularVelocity", state.omega.value()}};
}

void from_json(const json& j, Trajectory& traj) {
  traj = Trajectory(j.get<std::vector<Trajectory::State>>());
}

void to_json(json& j, const Trajectory& traj) {
  j = json{traj.m_states};
}
