// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <string>
#include <string_view>

#include <wpi/DataLog.h>

#include "util/log/TelemetryEntry.hpp"

class BoolTelemetryEntry {
 public:
  BoolTelemetryEntry(std::string_view name,
                       TelemetryLevel level = TelemetryLevel::kDebug);

  void Append(bool value, int64_t timestamp = 0);

 private:
  const std::string m_name;
  wpi::log::DoubleLogEntry m_logEntry;
  TelemetryLevel m_level;
};
