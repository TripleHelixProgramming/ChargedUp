// #include "util/TrajectoryManager.h"

// #include <filesystem>
// #include <fstream>
// #include <string_view>
// #include <vector>

// // #include <wpi/json.h>

// #include <frc/Filesystem.h>

// #include "util/Trajectory.h"

// // using wpi::json;
// using namespace std::filesystem;

// Trajectory& TrajectoryManager::GetTrajectory(const std::string& pathName) {
//   return m_trajectories[pathName];
// }

// void TrajectoryManager::LoadTrajectories() {
//   path trajDir =
//       path(frc::filesystem::GetDeployDirectory());
//   // for (const auto& file : directory_iterator(trajDir)) {
//   //   file.path().filename;
//   //   if (file.path().filename.ends_with(".json")) {
//   //     m_trajectories.insert({"", LoadTrajectory(file.path())});
//   //   }
//   // }
// }

// void TrajectoryManager::LoadTrajectory(const std::filesystem::path& trajPath) {
//   // std::ifstream fileStream(trajPath);
//   // std::stringstream buffer;
//   // buffer << fileStream.rdbuf();
//   // auto parsed = json::parse(buffer.str());
// }