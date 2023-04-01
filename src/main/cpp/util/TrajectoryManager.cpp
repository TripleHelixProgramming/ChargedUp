// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/TrajectoryManager.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <frc/Filesystem.h>
#include <wpi/json.h>

#include "util/Trajectory.hpp"

using std::filesystem::directory_iterator;
using std::filesystem::path;
using wpi::json;

const Trajectory& TrajectoryManager::GetTrajectory(const std::string& name) {
  return s_instance.m_trajectories.at(name);
}

std::map<std::string, Trajectory> TrajectoryManager::LoadTrajectories() {
  std::map<std::string, Trajectory> trajectories;
  path trajDir =
      path(frc::filesystem::GetDeployDirectory()) / path("trajectories");
  for (const auto& file : directory_iterator(trajDir)) {
    auto filename = file.path().filename().string();
    if (filename.ends_with(".json")) {
      auto trajname = filename.substr(0, filename.length() - 5);
      trajectories.insert({std::move(trajname), LoadFile(file.path())});
    }
  }
  return trajectories;
}

Trajectory TrajectoryManager::LoadFile(const path& trajPath) {
  std::ifstream fileStream(trajPath);
  if (!fileStream.fail()) {
    std::stringstream buffer;
    buffer << fileStream.rdbuf();
    auto parsed = json::parse(buffer.str());
    return json(parsed);
  } else {
    throw std::runtime_error("Error loading trajectory file.");
  }
}

TrajectoryManager::TrajectoryManager() : m_trajectories{LoadTrajectories()} {}

TrajectoryManager TrajectoryManager::s_instance{};
