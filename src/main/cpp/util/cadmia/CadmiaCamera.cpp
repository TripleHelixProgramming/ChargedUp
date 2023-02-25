// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/cadmia/CadmiaCamera.hpp"

#include <photonlib/PhotonTrackedTarget.h>

#include "util/cadmia/CadmiaTrackedTarget.hpp"

using namespace cadmia;
using namespace units;

CadmiaCamera::CadmiaCamera(std::string_view name) : m_name{name} {
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("cadmia");
  m_subscriber = table->GetDoubleArrayTopic(m_name).Subscribe({});
}

CadmiaPipelineResult CadmiaCamera::GetResult() {
  // Pipeline result arrives from coprocessor as
  // [id0, x0, y0, x1, y1, x2, y2, x3, y3, ...]
  auto result = m_subscriber.GetAtomic();
  auto time = result.time;
  auto compressedResults = result.value;
  m_targets.clear();

  for (size_t index = 0; index < compressedResults.size(); index += 9) {
    wpi::SmallVector<std::pair<double, double>, 4> corners;
    for (int corner_index = 0; corner_index < 4; ++corner_index) {
      corners.push_back({compressedResults[index + 2 * corner_index + 1],
                         compressedResults[index + 2 * corner_index + 2]});
    }
    m_targets.push_back(
        CadmiaTrackedTarget{(int)compressedResults[index],  // Fiducial ID
                            corners});
  }

  return CadmiaPipelineResult{second_t{(double)time}, m_targets};
}
