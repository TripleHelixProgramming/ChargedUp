// #pragma once

// #include <vector>

// #include <frc/geometry/Pose2d.h>
// #include <frc/geometry/Twist2d.h>

// #include <Eigen/Core>
// #include <units/angular_velocity.h>
// #include <units/velocity.h>
// #include <units/time.h>
// #include <units/angle.h>

// #include <wpi/json.h>

// class Trajectory {
//  public:
//   struct State {
//     units::second_t t;
//     frc::Pose2d pose;
//     units::meters_per_second_t vx;
//     units::meters_per_second_t vy;
//     units::radians_per_second_t omega;

//     State Interpolate(const State& other, units::second_t newT) const;
//   };

//   // void from_json(const wpi::json& j, State& sample);
  
//   Trajectory(std::vector<State> states);

//   State Sample(units::second_t t);

//   units::second_t GetTotalTime();
  
//  private:
//   std::vector<State> m_states; 
// };

// // void from_json(const wpi::json& j, Trajectory& traj);