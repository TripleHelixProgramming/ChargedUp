// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <memory>
#include <vector>

#include "util/trajectory/constraint/TrajectoryConstraint.h"

class TrajectoryConfig {
 public:
  template <typename T, typename = std::enable_if_t<std::is_base_of_v<TrajectoryConstraint, T>>>
  void ApplyConstraint(T constraint) {
    m_constraints.emplace_back(std::make_unique<T>(constraint));
  }

  const std::vector<std::unique_ptr<TrajectoryConstraint>>& Constraints() const {
    return m_constraints;
  }

 private:
  std::vector<std::unique_ptr<TrajectoryConstraint>> m_constraints;
};
