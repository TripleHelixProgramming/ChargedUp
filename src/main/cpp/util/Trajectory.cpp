#include "util/Trajectory.h"

#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/angle.h>
#include <units/base.h>

#include <wpi/json.h>

using namespace units;
using namespace frc;

Trajectory::State Trajectory::State::Interpolate(const Trajectory::State& other, second_t newT) const {
  double scale = ((newT - t) / (other.t - t)).value();
  return Trajectory::State{ 
    newT,
    pose.Exp(pose.Log(other.pose) * scale),
    (other.vx - vx) * scale + vx,
    (other.vy - vy) * scale + vy,
    (other.omega - omega) * scale + omega
  };
  return other;
}

// void from_json(const wpi::json& j, Trajectory::State& sample) {
//   sample.t = second_t{j.at("timestamp")};
// //   sample.pose = Pose2d(Translation2d(meter_t{j.at("x")}, meter_t{j.at("y")}),
// //                        Rotation2d(radian_t{j.at("heading")}));
// //   sample.vx = j.at("velocityX");
// //   sample.vy = j.at("velocityY");
// //   sample.omega = j.at("angularVelocity");
// }

Trajectory::Trajectory(std::vector<Trajectory::State> states) : m_states{states} {}

Trajectory::State Trajectory::Sample(second_t t) {
  if (t.value() < m_states[0].t.value()) {
    return m_states[0];
  }
  if (t.value() > GetTotalTime().value()) {
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

second_t Trajectory::GetTotalTime() {
  return m_states.back().t;
}

// void Trajectory::from_json(const wpi::json& j, Trajectory& traj) {
  
// }