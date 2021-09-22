/**
*
*/

#include "target_estimation/types/projectile.hpp"

#ifdef LOGGER_ON
#include <rt_logger/rt_logger.h>
using namespace rt_logger;
#endif

using namespace std;

// ----------------------------
// TargetProjectile
// ----------------------------
TargetProjectile::TargetProjectile(const unsigned int& id,
                     const double& dt0,
                     const Eigen::MatrixXd&   Q,
                     const Eigen::MatrixXd&   R,
                     const Eigen::MatrixXd&   P0,
                     const Eigen::Vector7d&   p0,
                     const double& t0) :
  TargetInterface(id,P0,t0)
{
  n_ = static_cast<unsigned int>(Q.rows()); // Number of states
  m_ = static_cast<unsigned int>(R.rows()); // Number of measurements

  // Supported case:
  // n = 9: [x y z \dot{x} \dot{y} \dot{z} \ddot{x} \ddot{y} \ddot{z}]
  assert(n_ == 9);
  assert(m_ <= n_);
  assert(dt0>=0.0);

  acceleration_on_ = true;

  A_.resize(n_, n_);
  updateA(dt0);

  // Output matrix
  C_.resize(m_, n_);
  C_.setZero();
  C_.diagonal() = Eigen::VectorXd::Ones(m_);

  // Construct the estimator
  estimator_.reset(new LinearKalmanFilter(A_, C_, Q, R, P0));

  // Initialize the state
  x_ = Eigen::VectorXd::Zero(n_);

  PROJECTILE_TARGET_pos(x_) = p0.segment(0,3);
  PROJECTILE_TARGET_vel(x_) << 0.0, 0.0, 0.0;
  PROJECTILE_TARGET_acc(x_) << 0.0, 0.0, -GRAVITY;

  std::cout << std::endl;
  std::cout << "*** TargetProjectile ***" << std::endl;
  std::cout << "*** Initialization ***" << std::endl;
  std::cout << "n: "<< n_ << std::endl;
  std::cout << "m: "<< m_ << std::endl;
  std::cout << "dt0: "<< dt0 << std::endl;
  std::cout << "t0: "<< t0 << std::endl;
  std::cout << "x0:" << std::endl;
  std::cout << x_.transpose() << std::endl;
  std::cout << std::endl;
  std::cout << "*** System Matrices ***" << std::endl;
  std::cout << "A:" << std::endl;
  std::cout << A_ << std::endl;
  std::cout << "C:" << std::endl;
  std::cout << C_ << std::endl;
  std::cout << std::endl;
  std::cout << "*** Covariance Matrices ***" << std::endl;
  std::cout << "Q:" << std::endl;
  std::cout << Q << std::endl;
  std::cout << "R:" << std::endl;
  std::cout << R << std::endl;
  std::cout << "P0:" << std::endl;
  std::cout << P0 << std::endl;
  if(!acceleration_on_)
    std::cout << "Assuming constant velocities" << std::endl;

  estimator_->init(x_);

  updateTargetState();
}

void TargetProjectile::addMeasurement(const double& dt, const Eigen::Vector7d& meas)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt);

  vector3d_tmp_ << meas(0), meas(1), meas(2); // xyz

  std::dynamic_pointer_cast<LinearKalmanFilter>(estimator_)->update(vector3d_tmp_, A_);

  updateTargetState();
  updateTime(dt);
  updateMeasurement(meas);
}

void TargetProjectile::update(const double& dt)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt);

  std::dynamic_pointer_cast<LinearKalmanFilter>(estimator_)->update(A_);

  updateTargetState();
  updateTime(dt);
}

void TargetProjectile::updateA(const double& dt)
{
  // A matrix using this dt
  // Discrete LTI Target motion with gravity acceleration acting on the z axis
  A_.setZero();
  A_.diagonal()     = Eigen::VectorXd::Ones(n_);
  A_.diagonal(n_/2) = Eigen::VectorXd::Ones(n_/2) * dt;
  if(acceleration_on_)
    A_.diagonal(n_) = Eigen::VectorXd::Ones(n_) * 0.5 * dt * dt;
}

void TargetProjectile::updateTargetState()
{
  // Read the estimated state
  x_ = estimator_->getState();
  // Read the covariance
  P_ = estimator_->getP();
  // Set our Target's state
  position_ = PROJECTILE_TARGET_pos(x_);
  POSE_pos(pose_) = position_;
  T_.translation() = position_;
  T_.linear() = Eigen::Matrix3d::Identity();

  TWIST_linear(twist_) = PROJECTILE_TARGET_vel(x_);
  TWIST_angular(twist_) << 0.0, 0.0, 0.0;

  ACCELERATION_linear(acceleration_) = PROJECTILE_TARGET_acc(x_);
  ACCELERATION_angular(acceleration_) << 0.0, 0.0, 0.0;
}

Eigen::Vector7d TargetProjectile::getEstimatedPose(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);

  POSE_pos(vector6d_tmp_) = position_ + TWIST_linear(twist_)*(t1-t_) + 0.5 * ACCELERATION_linear(acceleration_)*(t1-t_)*(t1-t_);

  quaterniond_tmp_.setIdentity();
  POSE_pos(vector7d_tmp_)  = POSE_pos(vector6d_tmp_);
  POSE_quat(vector7d_tmp_) = quaterniond_tmp_.coeffs();

  return vector7d_tmp_;
}

Eigen::Vector6d TargetProjectile::getEstimatedTwist(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);
  return twist_ + acceleration_*(t1-t_);
}