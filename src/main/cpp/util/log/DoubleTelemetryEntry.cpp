#include "util/log/DoubleTelemetryEntry.h"

#include <string_view>
#include <frc/smartdashboard/SmartDashboard.h>
#include "util/log/TelemetryEntry.h"

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>

#include "Constants.h"

DoubleTelemetryEntry::DoubleTelemetryEntry(std::string_view name, TelemetryLevel level)
  : m_name(name), m_logEntry(frc::DataLogManager::GetLog(), m_name), m_level(level) {}

void DoubleTelemetryEntry::Append(double value) {
  m_logEntry.Append(value, 0);
  if (kTelemetryLevel >= m_level) {
    frc::SmartDashboard::PutNumber(m_name, value);
  }
}