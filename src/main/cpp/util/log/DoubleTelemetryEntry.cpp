// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/log/DoubleTelemetryEntry.h"

#include <string_view>

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/DataLog.h>

#include "Constants.h"
#include "util/log/TelemetryEntry.h"

DoubleTelemetryEntry::DoubleTelemetryEntry(std::string_view name, TelemetryLevel level)
    : m_name(name), m_logEntry(frc::DataLogManager::GetLog(), m_name), m_level(level) {
  if (kTelemetryLevel >= m_level) {
    frc::SmartDashboard::PutNumber(m_name, 0.0); // make sure it shows up in smartdashboard
  }
}

void DoubleTelemetryEntry::Append(double value, int64_t timestamp) {
  m_logEntry.Append(value, timestamp);
  if (kTelemetryLevel >= m_level) {
    frc::SmartDashboard::PutNumber(m_name, value);
  }
}
