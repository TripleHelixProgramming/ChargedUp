// Copyright (c) FRC Team 2363. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/src/Core/Matrix.h>

namespace graph {

/**
 * Returns the skew symmetric operator of the given vector w
 *
 *   [  0   -w_z   w_y]
 *   [ w_z    0   -w_x]
 *   [-w_y   w_x    0 ]
 */
Eigen::Matrix3d Skew(Eigen::Vector3d w);

/**
 * Computes numerical jacobian of given function
 */
Eigen::SparseMatrix<double> NumericalJacobian(
    std::function<Eigen::VectorXd(Eigen::VectorXd)> f, Eigen::VectorXd x);

/**
 * Computes the optimal translation vector T and rotation matrix R as the
 * solution to the problem:
 *
 *   min |h(T, R) - z|
 *
 * Note this is a least-squares problem minimizing the error between predicted
 * and measured sensor readings.
 *
 * Implementation based on https://www.cs.cmu.edu/~kaess/pub/Dellaert17fnt.pdf
 *
 * @param reprojectionFunction Reprojection function h(T, R) which predicts
 * sensor readings from a given pose.
 * @param measurement Measured sensor readings z.
 */
void OptimizePose(
    std::function<Eigen::VectorXd(Eigen::Vector3d, Eigen::Matrix3d)>
        reprojectionFunction,
    Eigen::VectorXd measurement);

}  // namespace graph
