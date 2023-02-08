#include "commands/autos/North2ConeCharge.h"
#include "commands/DriveTrajectory.hpp"

North2ConeCharge::North2ConeCharge(SwerveDrive* drive, const TrajectoryManager* trajManager)
    : m_drive(drive), m_trajManager(trajManager) {
  AddCommands(
      DriveTrajectory(m_drive, &m_trajManager->GetTrajectory("north_place_grid3x1")),
      DriveTrajectory(m_drive, &m_trajManager->GetTrajectory("north_pick4")),
      DriveTrajectory(m_drive, &m_trajManager->GetTrajectory("north_place_grid3x3")),
      DriveTrajectory(m_drive, &m_trajManager->GetTrajectory("north_charging_station")));
}
