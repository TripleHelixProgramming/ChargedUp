#include <gtest/gtest.h>

#include <Eigen/Core>
// #include "util/graph/FactorGraphSolver.hpp"

TEST(FactorGraphTest, Translation) {
  std::cout << "Factor graph" << std::endl;

  Eigen::VectorXd measurement{3};
  measurement << 1, 1, 1;

  auto h = [](Eigen::Vector3d T, Eigen::MatrixXd R) {
    return R * Eigen::VectorXd::Ones(3);
  };

//   graph::OptimizePose(h, measurement);
}
