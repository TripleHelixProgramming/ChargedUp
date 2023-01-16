#include "util/TrajectoryManager.h"

#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string_view>
#include <vector>

#include <wpi/json.h>

#include <frc/Filesystem.h>

#include "util/Trajectory.h"

using wpi::json;
using namespace std::filesystem;

Trajectory& TrajectoryManager::GetTrajectory(const std::string& pathName) {
  return m_trajectories[pathName];
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

Trajectory TrajectoryManager::LoadFile(const std::filesystem::path& trajPath) {
  std::ifstream fileStream(trajPath);
  std::stringstream buffer;
  buffer << fileStream.rdbuf();
  auto parsed = json::parse(buffer.str());
}