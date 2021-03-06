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

#ifndef TARGET_INTERFACE_HPP
#define TARGET_INTERFACE_HPP

#include <map>
#include <string>
#include <fstream>
#include <mutex>
#include <iostream>
#include <Eigen/Dense>
#include <memory>

#include "target_estimation/kalman.hpp"
#include "target_estimation/utils.hpp"
#include "target_estimation/geometry.hpp"

/**
 * @brief The TargetInterface class
 * Generic interface for the different implementations.
 */
class TargetInterface
{

public:

  typedef std::shared_ptr<TargetInterface> Ptr;

  TargetInterface(const unsigned int& id, const Eigen::MatrixXd P0, const double& t0);

  TargetInterface() : id_(-1) {};

  virtual ~TargetInterface();

  /**
     * @brief addMeasurement Add a measured value to the estimator and perform an update step
     * of the filters
     * @param dt
     * @param meas
     */
  virtual void addMeasurement(const double& dt, const Eigen::Vector7d& meas) = 0;

  /**
    * @brief update Perform a predict step of the filters when no measurement given
    * @param dt
    */
  virtual void update(const double& dt) = 0;

  /**
     * @brief getEstimatedPose
     * @param t
     * @return Target's estimated pose at time t [x y z qx qy qz qw]
     * NOTE: if not implemented in subclass, return current value
     */
  virtual Eigen::Vector7d getEstimatedPose(const double& t);

  /**
     * @brief getEstimatedTwist
     * @param t
     * @return Target's estimated twist at time t
     * NOTE: if not implemented in subclass, return current value
     */
  virtual Eigen::Vector6d getEstimatedTwist(const double& t);

  /**
     * @brief getEstimatedAcceleration
     * @param t
     * @return Target's estimated acceleration at time t
     * NOTE: if not implemented in subclass, return current value
     */
  virtual Eigen::Vector6d getEstimatedAcceleration(const double& t);

  /**
     * @brief getPeriodEstimate
     * @return estimated rotation period
     */
  double getPeriodEstimate();

  /**
     * @brief getTime
     * @return internal time
     */
  double getTime();

  /**
     * @brief getEstimatedTransform
     * @return last estimated Target's pose
     */
  const Eigen::Isometry3d& getEstimatedTransform();

  /**
     * @brief getEstimatedPose
     * @return last estimated Target's pose
     */
  const Eigen::Vector7d& getEstimatedPose();

  /**
     * @brief getEstimatedTwist
     * @return last estimated Target's linear and angular velocity
     */
  const Eigen::Vector6d& getEstimatedTwist();

  /**
     * @brief getEstimatedAcceleration
     * @return last estimated Target's acceleration
     */
  const Eigen::Vector6d& getEstimatedAcceleration();

  /**
     * @brief getMeasuredPose
     * @return last measured Target's pose
     */
  const Eigen::Vector7d& getMeasuredPose();

  /**
     * @brief getID
     * @return Target's ID
     */
  unsigned int getID() const {return id_;}

  /**
     * @brief getN
     * @return n state dimension
     */
  unsigned int getN() const {return n_;}

  /**
     * @brief getM
     * @return m measurement dimension
     */
  unsigned int getM() const {return m_;}

  /**
     * @brief getEstimator
     * @return get a constant reference to the kalman filter
     */
  KalmanFilterInterface* getEstimator() const {return estimator_.get();}

  /**
     * @brief getNumberMeasurements
     * @return the number of measurements done so far
     */
  long long getNumberMeasurements() const {return n_meas_;}

  /**
     * @brief log Log data through ros
     */
  void log();

  /**
     * @brief print Target's info to screen
     */
  void printInfo();

protected:

  /**
     * @brief updateTargetState Update the internal variables representing
     * the state of the target
     */
  virtual void updateTargetState() = 0;

  /**
     * @brief updateMeasurement Update the measurement and the measurement counter
     */
  void updateMeasurement(const Eigen::Vector7d& meas);

  /**
     * @brief updateTime  Update internal time variable
     */
  void updateTime(const double& dt);

  /**
     * @brief class_name_ name of the class
     */
  std::string class_name_;

  /**
     * @brief n_ Number of states
     */
  unsigned int n_;

  /**
     * @brief m_ Number of measurements
     */
  unsigned int m_;

  /**
     * @brief id_ ID number
     */
  int id_;

  /**
     * @brief t_ Time of last estimate
     */
  double t_;

  /**
     * @brief pose_ Estimated pose transform
     */
  Eigen::Isometry3d T_;

  /**
     * @brief twist_ Estimated twist
     */
  Eigen::Vector6d twist_;

  /**
     * @brief acceleration_ Estimated acceleration
     */
  Eigen::Vector6d acceleration_;

  /**
     * @brief pose_ Estimated pose in terms of [x y z roll pitch yaw]
     */
  Eigen::Vector6d pose_internal_;

  /**
     * @brief measured_pose_ Measured pose
     */
  Eigen::Vector7d measured_pose_;

  /**
     * @brief P_ Kalman Filter covariance matrix
     */
  Eigen::MatrixXd P_;

  /**
     * @brief x_ Estimated state, with vectors of pose/twist/acceleration
     */
  Eigen::VectorXd x_;

  /**
     * @brief estimator_ State estimator
     */
  KalmanFilterInterface::Ptr estimator_;

  /**
     * @brief n_meas_ How many measurements we've made
     */
  long long n_meas_;

  /**
     * @brief data_lock_ Used to prevent concurrent access to variables
     */
  std::mutex data_lock_;

  /**
     * @brief C_ output matrix
     */
  Eigen::MatrixXd C_;

  /**
     * @brief A_ dynamical system matrix
     */
  Eigen::MatrixXd A_;

  /**
     * @brief Temporary variables
     */
  Eigen::Vector3d vector3d_tmp_;
  Eigen::Vector6d vector6d_tmp_;
  Eigen::Vector7d vector7d_tmp_;
  Eigen::Isometry3d isometry3d_tmp_;
  Eigen::Quaterniond quaterniond_tmp_;
};

#endif
