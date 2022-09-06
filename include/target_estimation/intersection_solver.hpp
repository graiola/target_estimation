/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/
#ifndef INTERSECTION_SOLVER_HPP
#define INTERSECTION_SOLVER_HPP

#include <cmath>
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <unsupported/Eigen/Polynomials>

#include "target_estimation/target_manager_ros.hpp"

/* Solve n-th order polynomial

    Pass in the coefficients with the zero-th order term first in the vector
    i.e. for a0+a1*x+a2*x^2=0 pass in [a0 a1 a2]
*/

class Solver
{

public:

  Solver(){}

  double lowestRealRoot(const Eigen::VectorXd &coeffs);

private:

  Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;

};

/**
 * @brief The IntersectionSolver class
 * Calculate the intersection of estimated target poses with geometric shapes
 */
class IntersectionSolver
{

public:

  typedef std::shared_ptr<IntersectionSolver> Ptr;

  IntersectionSolver(TargetManager::Ptr target_manager, const unsigned int filters_length = 250);

  /**
     * @brief getIntersectionTimeWithSphere
     * @param id
     * @param t
     * @param origin
     * @param radius
     * @return the time the Target(id) will first intersect with a sphere at the given origin and radius, or -1 if it will not happen.
     */
  double getIntersectionTimeWithSphere(const unsigned int& id, const double& t1, const Eigen::Vector3d& origin, const double& radius);

  /**
     * @brief getIntersectionPoseWithSphere
     * @param id
     * @param t
     * @param pos_th maximum allowable position error [m]
     * @param ang_th maximum allowable angular error [rad]
     * @param origin
     * @param radius
     * @param intersection_pose
     * @return true if an intersection pose of Target(id) with a sphere at the given origin and radius exists, false otherwise.
     */
  bool getIntersectionPoseWithSphere(const unsigned int& id, const double& t1, const double& pos_th, const double& ang_th,
                                     const Eigen::Vector3d& origin, const double& radius, Eigen::Vector7d& intersection_pose);

  /**
   * @brief getIntersectionPoseAndtimeWithSphere
   * @param id
   * @param t1
   * @param pos_th
   * @param ang_th
   * @param origin
   * @param radius
   * @param intersection_pose
   * @param delta_intersect_t
   * @return true if an intersection pose of Target(id) with a sphere at the given origin and radius exists, false otherwise.
   */
  bool getIntersectionPoseAndtimeWithSphere(const unsigned int& id, const double& t1, const double& pos_th, const double& ang_th,
                                            const Eigen::Vector3d& origin, const double& radius,
                                            Eigen::Vector7d& intersection_pose, double& delta_intersect_t);


private:

    /**
       * @brief target_manager_ Target manager
       */
  TargetManager::Ptr target_manager_;

  /**
     * @brief solver_ Polynomial solver
     */
  Solver solver_;

  /**
     * @brief Moving average filters
     */
  MovingAvgFilter::Ptr pos_error_filter_;
  MovingAvgFilter::Ptr ang_error_filter_;

  /**
     * @brief Temporary variables
     */
  Eigen::Vector3d pos_tmp_;
  Eigen::Vector3d vel_tmp_;
  Eigen::Vector3d acc_tmp_;
  Eigen::Quaterniond q1_tmp_;
  Eigen::Quaterniond q2_tmp_;

  /**
     * @brief Previous saved intersection pose
     */
  Eigen::Vector7d intersection_pose_prev_;

  /**
  * @brief Polynomial coefficients
  */
  Eigen::VectorXd coeff_;

};

#endif

