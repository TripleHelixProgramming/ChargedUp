// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <span>
#include <string_view>
#include "util/cadmia/CadmiaTrackedTarget.hpp"
#include <networktables/DoubleArrayTopic.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <units/time.h>

namespace cadmia {

struct CadmiaPipelineResult {
  units::second_t time;
  std::span<CadmiaTrackedTarget> targets;
};

class CadmiaCamera {
 public:
  CadmiaCamera(std::string_view name);

  CadmiaPipelineResult GetResult();

 private:
  std::string_view m_name;
  nt::DoubleArraySubscriber m_subscriber;
};

}