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

#ifndef ANGULAR_VELOCITIES_HPP
#define ANGULAR_VELOCITIES_HPP

#include <map>
#include <string>
#include <fstream>
#include <mutex>
#include <iostream>
#include <Eigen/Dense>
#include <memory>

#include "target_estimation/target_interface.hpp"

/**
 * @brief The TargetAngularVelocities class
 * Extended implementation of the Kalman Filter with state variable defined as:
 * [x y z \psi \theta \phi \dot{x} \dot{y} \dot{z} \omega_x \omega_y \omega_z]
 */
class TargetAngularVelocities : public TargetInterface
{

public:

  typedef std::shared_ptr<TargetAngularVelocities> Ptr;

  TargetAngularVelocities(const unsigned int& id,
                   const double& dt0,
                   const double& t0,
                   const Eigen::MatrixXd&   Q,
                   const Eigen::MatrixXd&   R,
                   const Eigen::MatrixXd&   P0,
                   const Eigen::Vector7d&   p0,
                   const Eigen::Vector6d&   v0,
                   const Eigen::Vector6d&   a0);

  /**
     * @brief addMeasurement Add a measured value to the estimator and perform an update step
     * of the filters
     * @param dt
     * @param meas
     */
  virtual void addMeasurement(const double& dt, const Eigen::Vector7d& meas) override;


  /**
    * @brief update Perform a predict step of the filters, i.e. no measurement given
    * @param dt
    */
  virtual void update(const double& dt) override;

  /**
     * @brief getEstimatedPose
     * @param t
     * @return Target's estimated pose at time t
     */
  virtual Eigen::Vector7d getEstimatedPose(const double& t) override;

private:

  /**
     * @brief updateTargetState Update the internal variables representing
     * the state of the target
     */
  virtual void updateTargetState() override;

  void updateA(const double& dt, const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega);

  /**
     * @brief f non-linear sate transition function
     * @param x state vector
     * @param dt sample time
     * @return updated state
     */
  Eigen::VectorXd f(const Eigen::VectorXd& x, const double& dt);


  /**
     * @brief h non-linear output function
     * @param x state vector
     * @return updated output
     */
  Eigen::VectorXd h(const Eigen::VectorXd& x);

  /**
     * @brief Identity matrices
     */
  Eigen::Matrix3d I3_;

  /**
     * @brief Temporary variables
     */
  Eigen::VectorXd vectorXd_tmp_;

  /**
     * @brief EarInv_ Inverse of the Analytical Jacobian
     */
  Eigen::Matrix3d EarInv_;

  /**
   * @brief y_
   */
  Eigen::Vector6d y_;

  /**
     * @brief meas_internal_ Internal representation of the target's measurement in terms
     * of [x y z roll pitch yaw]
     */
  Eigen::Vector3d meas_rpy_internal_;
};

#endif
