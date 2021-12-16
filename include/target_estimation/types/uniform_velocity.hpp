/**
*
*/

#ifndef UNIFORM_VELOCITY_HPP
#define UNIFORM_VELOCITY_HPP

#include <map>
#include <string>
#include <fstream>
#include <mutex>
#include <iostream>
#include <Eigen/Dense>
#include <memory>

#include "target_estimation/target_interface.hpp"

/**
 * @brief The TargetUniformVelocity class
 * Linear implementation of a kalman filter describing a uniform rectilinear motion (URM) with state variable defined as:
 * [x y z \dot{x} \dot{y} \dot{z}]
 */
class TargetUniformVelocity : public TargetInterface
{

public:

  typedef std::shared_ptr<TargetUniformVelocity> Ptr;

  TargetUniformVelocity(const unsigned int& id,
            const double& dt0,
            const double& t0,
            const Eigen::MatrixXd&   Q,
            const Eigen::MatrixXd&   R,
            const Eigen::MatrixXd&   P0,
            const Eigen::Vector7d&   p0,
            const Eigen::Vector6d&   v0,
            const Eigen::Vector6d&   a0
            );

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
     * @brief updateA
     * @param dt
     */
  void updateA(const double& dt);

  /**
     * @brief getEstimatedPose
     * @param t
     * @return Target's estimated pose at time t
     */
  virtual Eigen::Vector7d getEstimatedPose(const double& t) override;

  /**
     * @brief getEstimatedTwist
     * @param t
     * @return Target's estimated twist at time t
     */
  virtual Eigen::Vector6d getEstimatedTwist(const double& t) override;

private:

  /**
     * @brief updateTargetState Update the internal variables representing
     * the state of the target
     */
  virtual void updateTargetState() override;

};

#endif