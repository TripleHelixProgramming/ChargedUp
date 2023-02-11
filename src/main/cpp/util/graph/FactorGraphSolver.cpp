// Copyright (c) FRC Team 2363. All Rights Reserved.

#include "util/graph/FactorGraphSolver.hpp"

#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <Eigen/src/SparseCore/SparseMatrix.h>
#include <unsupported/Eigen/MatrixFunctions>

Eigen::Matrix3d graph::Skew(Eigen::Vector3d w) {
  Eigen::Matrix3d W = Eigen::MatrixXd::Zero(3, 3);
  W(1, 0) = w.z();
  W(0, 1) = -w.z();
  W(2, 0) = -w.y();
  W(0, 2) = w.y();
  W(2, 1) = w.x();
  W(1, 2) = -w.x();
  return W;
}

Eigen::SparseMatrix<double> graph::NumericalJacobian(
    std::function<Eigen::VectorXd(Eigen::VectorXd)> f, Eigen::VectorXd x) {
  constexpr double h = 1e-5;

  std::vector<Eigen::Triplet<double>> triplets;

  for (int index = 0; index < x.rows(); ++index) {
    Eigen::VectorXd forwardDelta = x;
    x(index) += h;
    Eigen::VectorXd backwardDelta = x;
    x(index) -= h;
    Eigen::VectorXd col = (f(forwardDelta) - f(backwardDelta)) / (2 * h);
    for (int row = 0; row < col.rows(); ++row) {
      if (col(row) != 0.0) {
        triplets.emplace_back(row, index, col(row));
      }
    }
  }

  Eigen::SparseMatrix<double> J{f(x).rows(), x.rows()};
  J.setFromTriplets(triplets.begin(), triplets.end());
  return J;
}

void graph::OptimizePose(
    std::function<Eigen::VectorXd(Eigen::Vector3d, Eigen::Matrix3d)>
        reprojectionFunction,
    Eigen::VectorXd measurement) {
  std::cout << "begin" << std::endl;
  Eigen::Vector3d T = Eigen::VectorXd::Zero(3);
  Eigen::Matrix3d R = Eigen::MatrixXd::Identity(3, 3);

  while (true) {
    // Form a local parameterization with the increment vector S ∈ ℝ⁶
    //
    //   S = [t]
    //       [w]
    //
    // Where
    //
    //   t ∈ ℝ³
    //   w ∈ ℝ³
    //
    //   T(t) = T₀ + t
    //   R(w) = R₀exp(J(w))
    //
    // J(w) is the skew symmetric matrix
    //
    //   [  0   -w_z   w_y]
    //   [ w_z    0   -w_x]
    //   [-w_y   w_x    0 ]
    //
    // The measurement function is redefined in the local paramterization.
    //
    //   h(S) = h(T(t), R(w))
    //
    // This avoids singularities and other nonlinear behavior with global
    // paremeterizations such as euler angles.
    auto localParameterization = [&](Eigen::VectorXd S) {
      Eigen::VectorXd t = S.segment(0, 3);
      Eigen::VectorXd w = S.segment(3, 3);
      return reprojectionFunction(T + t, R * Skew(w).exp());
    };

    std::cout << "Formed local parameterization" << std::endl;

    // f(S) = h(S) - z
    Eigen::VectorXd f = reprojectionFunction(T, R) - measurement;

    // Linearize Jacobian around S = 0
    Eigen::SparseMatrix<double> J =
        NumericalJacobian(localParameterization, Eigen::VectorXd::Zero(6));

    std::cout << "Computed evaluations" << std::endl;

    // Gauss-Newton method
    //
    // AᵀAΔ = Aᵀb
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver{J.transpose() *
                                                              J};
    Eigen::VectorXd step = solver.solve(-J.transpose() * f);

    Eigen::VectorXd t = step.segment(0, 3);
    Eigen::VectorXd w = step.segment(3, 3);

    // Increment global parameterization from local increment
    T += t;
    R = R * Skew(w).exp();

    if (step.norm() < 1e-6) {
      break;
    }
  }

  std::cout << "Translation: \n" << T << std::endl;
  std::cout << "Rotation: \n" << R << std::endl;
}
