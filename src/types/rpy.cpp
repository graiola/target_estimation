/**
*
*/

#include "target_estimation/types/rpy.hpp"

#ifdef LOGGER_ON
#include <rt_logger/rt_logger.h>
using namespace rt_logger;
#endif

using namespace std;

// ----------------------------
// TargetRpy
// ----------------------------
TargetRpy::TargetRpy(const unsigned int& id,
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

  // Supported cases:
  // n = 12: [x y z \psi \theta \phi
  //          \dot{x} \dot{y} \dot{z} \dot{\psi} \dot{\theta} \dot{\phi}]
  // n = 18: [x y z \psi \theta \phi
  //          \dot{x} \dot{y} \dot{z} \dot{\psi} \dot{\theta} \dot{\phi}
  //          \ddot{x} \ddot{y} \ddot{z} \ddot{\psi} \ddot{\theta} \ddot{\phi}]
  assert(n_ == 12 || n_ == 18);
  assert(m_ <= n_);
  assert(dt0>=0.0);

  if(n_ == 18)
    acceleration_on_ = true;
  else
    acceleration_on_ = false;

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
  pose7d2pose6d(p0,pose_internal_);

  RPY_TARGET_pose(x_)  = pose_internal_;

  std::cout << std::endl;
  std::cout << "*** TargetRpy ***" << std::endl;
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

void TargetRpy::addMeasurement(const double& dt, const Eigen::Vector7d& meas)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt);

  // Convert from 7d pose to 6d and
  // unwrap the angles because the estimator integrates over continuous angles
  POSE_pos(vector6d_tmp_) = POSE_pos(meas); // pos
  quaterniond_tmp_.coeffs() = POSE_quat(meas); // quat
  quaterniond_tmp_.normalize(); // Normalize the quaternion
  quatToRpy(quaterniond_tmp_,vector3d_tmp_);
  vector3d_tmp_ = unwrap(meas_rpy_internal_,vector3d_tmp_);
  POSE_rpy(vector6d_tmp_) = vector3d_tmp_;
  // Let's store the values
  meas_rpy_internal_ = vector3d_tmp_;

  std::dynamic_pointer_cast<LinearKalmanFilter>(estimator_)->update(vector6d_tmp_, A_);

  updateTargetState();
  updateTime(dt);
  updateMeasurement(meas);
}

void TargetRpy::update(const double& dt)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt);

  std::dynamic_pointer_cast<LinearKalmanFilter>(estimator_)->update(A_);

  updateTargetState();
  updateTime(dt);
}

void TargetRpy::updateA(const double& dt)
{
  // A matrix using this dt
  // Discrete LTI Target motion
  A_.setZero();
  A_.diagonal()     = Eigen::VectorXd::Ones(n_);
  A_.diagonal(n_/2) = Eigen::VectorXd::Ones(n_/2) * dt;
  if(acceleration_on_)
    A_.diagonal(n_) = Eigen::VectorXd::Ones(n_) * 0.5 * dt * dt;
}

void TargetRpy::updateTargetState()
{
  // Read the estimated state
  x_ = estimator_->getState();
  // Read the covariance
  P_ = estimator_->getP();
  // Set our Target's state
  position_ = RPY_TARGET_pos(x_);
  POSE_pos(pose_) = position_;
  pose_internal_ = RPY_TARGET_pose(x_);
  rpy_ = RPY_TARGET_rpy(x_);
  rpyToQuat(rpy_,q_);
  q_.normalize(); // Normalize the quaternion
  POSE_quat(pose_) = q_.coeffs();
  T_.translation() = position_;
  T_.linear() = q_.toRotationMatrix();

  TWIST_linear(twist_) = RPY_TARGET_vel(x_);
  // Convert from angular rates to angular velocities (omega)
  rpyToEarBase(rpy_,Ear_);
  TWIST_angular(twist_) = Ear_ * RPY_TARGET_rates(x_);

  if(acceleration_on_)
    acceleration_ = RPY_TARGET_acc(x_);
  else
    acceleration_.setZero();
}

Eigen::Vector7d TargetRpy::getEstimatedPose(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);

  vector6d_tmp_ = pose_internal_ + twist_*(t1-t_) + 0.5 * acceleration_*(t1-t_)*(t1-t_);
  rpyToQuat(RPY_TARGET_rpy(vector6d_tmp_),quaterniond_tmp_);
  quaterniond_tmp_.normalize(); // Normalize the quaternion
  POSE_pos(vector7d_tmp_)  = POSE_pos(vector6d_tmp_);
  POSE_quat(vector7d_tmp_) = quaterniond_tmp_.coeffs();

  return vector7d_tmp_;
}

Eigen::Vector6d TargetRpy::getEstimatedTwist(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);
  return twist_ + acceleration_*(t1-t_);
}
