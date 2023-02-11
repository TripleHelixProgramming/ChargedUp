#include <gtest/gtest.h>

// #include <Eigen/Core>
// #include "util/graph/FactorGraphSolver.hpp"

// Eigen::VectorXd reprojection(Eigen::Vector3d T, Eigen::Matrix3d R) {
//   return R * Eigen::Vector3d::Ones();
// }

// TEST(FactorGraphTest, Translation) {
//   std::cout << "Factor graph" << std::endl;

//   Eigen::VectorXd measurement;
//   measurement << 1, 1, 1;

//   std::function<Eigen::VectorXd(Eigen::Vector3d, Eigen::Matrix3d)> h = reprojection;

//   graph::OptimizePose(h, measurement);

//   EXPECT_EQ(0.0, 0.0);
// }
