// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/TrajectoryManager.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string_view>
#include <vector>

#include <frc/Filesystem.h>
#include <wpi/json.h>

#include "util/Trajectory.hpp"

using std::filesystem::directory_iterator;
using std::filesystem::path;
using wpi::json;

TrajectoryManager& TrajectoryManager::GetInstance() {
  return s_instance;
}

const Trajectory& TrajectoryManager::GetTrajectory(
    const std::string& name) const {
  return m_trajectories.at(name);
}

void TrajectoryManager::LoadTrajectories() {
  path trajDir =
      path(frc::filesystem::GetDeployDirectory()) / path("trajectories");
  for (const auto& file : directory_iterator(trajDir)) {
    std::string filepath = file.path().string();
    std::string filename = filepath.substr(filepath.find_last_of('/') + 1);
    if (filename.ends_with(".json")) {
      std::string trajname = filename.substr(0, filename.length() - 5);
      m_trajectories.insert({trajname, LoadFile(file.path())});
    }
  }
}

Trajectory TrajectoryManager::LoadFile(const path& trajPath) {
  std::ifstream fileStream(trajPath);
  std::stringstream buffer;
  buffer << fileStream.rdbuf();
  auto parsed = json::parse(buffer.str());
  return json(parsed);
}

TrajectoryManager::TrajectoryManager() {
  LoadTrajectories();
}

TrajectoryManager TrajectoryManager::s_instance{};
