// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <span>
#include <string_view>

#include <networktables/DoubleArrayTopic.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <units/time.h>

#include "photonlib/PhotonPoseEstimator.h"

namespace cadmia {

class CadmiaCamera {
 public:
  CadmiaCamera(std::string_view name);

  std::optional<photonlib::EstimatedRobotPose> GetResult();

 private:
  std::string_view m_name;
  nt::DoubleArraySubscriber m_subscriber;
};

}  // namespace cadmia
