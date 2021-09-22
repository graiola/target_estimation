/**
*
*/

#ifndef PROJECTILE_HPP
#define PROJECTILE_HPP

#include <map>
#include <string>
#include <fstream>
#include <mutex>
#include <iostream>
#include <Eigen/Dense>
#include <memory>

#include "target_estimation/target_interface.hpp"

/**
 * @brief The TargetProjectile class
 * Linear implementation of the Kalman Filter with state variable defined as:
 * [x y z \psi \theta \phi \dot{x} \dot{y} \dot{z} \dot{\psi} \dot{\theta} \dot{\phi}]
 * if we assume constant velocities, otherwise:
 * [x y z \psi \theta \phi \dot{x} \dot{y} \dot{z} \dot{\psi} \dot{\theta} \dot{\phi} \ddot{x} \ddot{y} \ddot{z} \ddot{\psi} \ddot{\theta} \ddot{\phi}]
 * Note1: the state vector is automatically defined by the dimension of the covariance matrices.
 * Note2: the derivative of the orientation angles are defined w.r.t the current frame, i.e.
 * they represent the angular rates. These are different from the angular velocities omega defined w.r.t
 * the body frame. The linear approximation works well for small angular rates, but it degenerates for
 * big angular changes. For this reason we implemented a version using the ekf to compute the orientation kinematics.
 */
class TargetProjectile : public TargetInterface
{

public:

  typedef std::shared_ptr<TargetProjectile> Ptr;

  TargetProjectile(const unsigned int& id,
            const double& dt0,
            const Eigen::MatrixXd&   Q,
            const Eigen::MatrixXd&   R,
            const Eigen::MatrixXd&   P0,
            const Eigen::Vector7d&   p0,
            const double& t0 = 0.0);

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
