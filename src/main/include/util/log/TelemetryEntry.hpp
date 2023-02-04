// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

/**
 * Level of detail for telemetry/logging to SmartDashboard.
 * Increasing int value of enums is increasing detail.
 */
enum class TelemetryLevel {
  /**
   * Only data needed for drivers will be put on SmartDashboard
   */
  kCompetition,
  /**
   * All data logged will be put on SmartDashboard
   */
  kDebug
};
