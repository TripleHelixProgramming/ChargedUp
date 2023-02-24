// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <frc/Filesystem.h>
#include <wpi/json.h>

#include "util/Trajectory.hpp"

class TrajectoryManager {
 public:
  TrajectoryManager();

  const Trajectory& GetTrajectory(const std::string& name) const;

 private:
  void LoadTrajectories();

  static Trajectory LoadFile(const std::filesystem::path& trajPath);

  std::map<std::string, Trajectory> m_trajectories;
};
