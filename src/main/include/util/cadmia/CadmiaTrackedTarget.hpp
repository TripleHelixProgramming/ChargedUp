// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <wpi/SmallVector.h>

namespace cadmia {

struct CadmiaTrackedTarget {
  int fiducialID;
  wpi::SmallVector<std::pair<double, double>, 4> corners;
};

}  // namespace cadmia
