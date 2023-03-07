// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/cadmia/CadmiaCamera.hpp"

#include <photonlib/PhotonTrackedTarget.h>
#include <optional>

#include "frc/geometry/Pose3d.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Translation3d.h"
#include "photonlib/PhotonPoseEstimator.h"
#include "units/angle.h"

using namespace cadmia;
using namespace units;

CadmiaCamera::CadmiaCamera(std::string_view name) : m_name{name} {
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("cadmia");
  m_subscriber = table->GetDoubleArrayTopic(m_name).Subscribe({});
}

std::optional<photonlib::EstimatedRobotPose> CadmiaCamera::GetResult() {
  // Pipeline result arrives from coprocessor as
  // [x, y, z, roll, pitch, yaw]
  auto result = m_subscriber.GetAtomic();
  auto time = result.time;
  auto compressedResults = result.value;

  if (result.value.size() == 6) {
    return photonlib::EstimatedRobotPose{
      frc::Pose3d(frc::Translation3d(
                    meter_t{compressedResults[0]},
                    meter_t{compressedResults[1]},
                    meter_t{compressedResults[2]}),
                  frc::Rotation3d(
                    radian_t{compressedResults[3]},
                    radian_t{compressedResults[4]},
                    radian_t{compressedResults[5]})),
      second_t{(double)time}
    };
  } else {
    return std::nullopt;
  }
}
