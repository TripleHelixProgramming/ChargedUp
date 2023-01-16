#pragma once

#include <filesystem>
#include <map>
#include <vector>

#include <wpi/json.h>

#include <frc/Filesystem.h>

#include "util/Trajectory.h"

class TrajectoryManager {
 public:
  Trajectory& GetTrajectory(const std::string& name);

  void LoadTrajectories();

private:
  static Trajectory LoadFile(const std::filesystem::path& trajPath);

  std::map<std::string, Trajectory> m_trajectories;
};