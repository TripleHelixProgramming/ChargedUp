// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/log/BoolTelemetryEntry.hpp"

#include <string_view>

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/DataLog.h>

#include "Constants.hpp"
#include "util/log/TelemetryEntry.hpp"

BoolTelemetryEntry::BoolTelemetryEntry(std::string_view name,
                                           TelemetryLevel level)
    : m_name(name),
      m_logEntry(frc::DataLogManager::GetLog(), m_name),
      m_level(level) {
  if (kTelemetryLevel >= m_level) {
    frc::SmartDashboard::PutBoolean(
        m_name, false);  // make sure it shows up in smartdashboard
  }
}

void BoolTelemetryEntry::Append(bool value, int64_t timestamp) {
  m_logEntry.Append(value, timestamp);
  if (kTelemetryLevel >= m_level) {
    frc::SmartDashboard::PutBoolean(m_name, value);
  }
}
