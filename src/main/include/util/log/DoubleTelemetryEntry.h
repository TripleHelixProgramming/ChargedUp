#pragma once

#include "util/log/TelemetryEntry.h"

#include <string_view>
#include <wpi/DataLog.h>

class DoubleTelemetryEntry {
 public:
  DoubleTelemetryEntry(std::string_view name, TelemetryLevel level = TelemetryLevel::kCompetition);

  void Append(double value);

 private:
  std::string_view m_name;
  wpi::log::DoubleLogEntry m_logEntry;
  TelemetryLevel m_level;
};