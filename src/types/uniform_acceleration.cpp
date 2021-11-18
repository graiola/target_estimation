/**
*
*/

#include "target_estimation/types/uniform_acceleration.hpp"

using namespace std;

// Helpers
#define STATE_pos(x)      x.segment(0,3)
#define STATE_vel(x)      x.segment(3,3)
#define STATE_acc(x)      x.segment(6,3)

// ----------------------------
// TargetUniformAcceleration
// ----------------------------
TargetUniformAcceleration::TargetUniformAcceleration(const unsigned int& id,
                     const double& dt0,
                     const double& t0,
                     const Eigen::MatrixXd&   Q,
                     const Eigen::MatrixXd&   R,
                     const Eigen::MatrixXd&   P0,
                     const Eigen::Vector7d&   p0,
                     const Eigen::Vector6d&   v0,
                     const Eigen::Vector6d&   a0) :
  TargetInterface(id,P0,t0)
{
  class_name_ = "TargetUniformAcceleration";

  n_ = static_cast<unsigned int>(Q.rows()); // Number of states
  m_ = static_cast<unsigned int>(R.rows()); // Number of measurements

  // Supported case:
  // n = 9: [x y z \dot{x} \dot{y} \dot{z} \ddot{x} \ddot{y} \ddot{z}]
  assert(n_ == 9);
  assert(m_ <= n_);
  assert(dt0>=0.0);

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

  STATE_pos(x_) = POSE_pos(p0);
  STATE_vel(x_) = TWIST_linear(v0);
  STATE_acc(x_) = ACCELERATION_linear(a0);

  estimator_->init(x_);

  updateTargetState();

  printInfo();
}

void TargetUniformAcceleration::addMeasurement(const double& dt, const Eigen::Vector7d& meas)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt);
  updateMeasurement(meas);

  vector3d_tmp_ << measured_pose_(0), measured_pose_(1), measured_pose_(2); // xyz

  std::dynamic_pointer_cast<LinearKalmanFilter>(estimator_)->update(vector3d_tmp_, A_);

  updateTargetState();
  updateTime(dt);
}

void TargetUniformAcceleration::update(const double& dt)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt);

  std::dynamic_pointer_cast<LinearKalmanFilter>(estimator_)->update(A_);

  updateTargetState();
  updateTime(dt);
}

void TargetUniformAcceleration::updateA(const double& dt)
{
  // A matrix using this dt
  // Discrete LTI Target motion
  A_.setZero();
  A_.diagonal()     = Eigen::VectorXd::Ones(n_);
  A_.diagonal(n_/3) = Eigen::VectorXd::Ones((n_*2)/3) * dt;
  A_.diagonal((n_*2)/3) = Eigen::VectorXd::Ones(n_/3) * 0.5 * dt * dt;
}

void TargetUniformAcceleration::updateTargetState()
{
  // Read the estimated state
  x_ = estimator_->getState();
  // Read the covariance
  P_ = estimator_->getP();
  // Update T
  T_.translation() = STATE_pos(x_);
  T_.linear() = Eigen::Matrix3d::Identity();
  // Update twist
  TWIST_linear(twist_) = STATE_vel(x_);
  TWIST_angular(twist_) << 0.0, 0.0, 0.0;
  // Update acceleration
  ACCELERATION_linear(acceleration_) = STATE_acc(x_);
  ACCELERATION_angular(acceleration_) << 0.0, 0.0, 0.0;
  // Update internal variables
  isometryToPose6d(T_,pose_internal_);
}

Eigen::Vector7d TargetUniformAcceleration::getEstimatedPose(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);

  POSE_pos(vector6d_tmp_) = T_.translation() + TWIST_linear(twist_)*(t1-t_) + 0.5 * ACCELERATION_linear(acceleration_)*(t1-t_)*(t1-t_);
  quaterniond_tmp_.setIdentity();
  POSE_pos(vector7d_tmp_)  = POSE_pos(vector6d_tmp_);
  POSE_quat(vector7d_tmp_) = quaterniond_tmp_.coeffs();

  return vector7d_tmp_;
}

Eigen::Vector6d TargetUniformAcceleration::getEstimatedTwist(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);
  return twist_ + acceleration_*(t1-t_);
}
