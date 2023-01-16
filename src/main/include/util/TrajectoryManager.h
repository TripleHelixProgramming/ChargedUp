// #pragma once

// #include <filesystem>
// #include <map>
// #include <string_view>
// #include <vector>

// #include <wpi/json.h>

// #include <frc/Filesystem.h>

// #include "util/Trajectory.h"

// class TrajectoryManager {
//  public:
//   Trajectory& GetTrajectory(const std::string& name);

//   void LoadTrajectories();

//   void LoadTrajectory(const std::filesystem::path& trajPath);
// private:
//   std::map<std::string, Trajectory> m_trajectories;
// };