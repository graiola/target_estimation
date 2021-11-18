/**
*
*/

#include "target_estimation/types/angular_rates.hpp"

using namespace std;

// Helpers
#define STATE_twist(x)           x.segment(6,6)
#define STATE_vel(x)             x.segment(6,3)
#define STATE_rates(x)           x.segment(9,3)
#define STATE_pos(x)             x.segment(0,3)
#define STATE_pose(x)            x.segment(0,6)
#define STATE_rpy(x)             x.segment(3,3)
#define STATE_acc(x)             x.segment(12,6)

// ----------------------------
// TargetAngularRates
// ----------------------------
TargetAngularRates::TargetAngularRates(const unsigned int& id,
                                       const double& dt0,
                                       const double& t0,
                                       const Eigen::MatrixXd&   Q,
                                       const Eigen::MatrixXd&   R,
                                       const Eigen::MatrixXd&   P0,
                                       const Eigen::Vector7d&   p0,
                                       const Eigen::Vector6d&   v0,
                                       const Eigen::Vector6d&   a0)
  : TargetInterface(id,P0,t0)
{
  class_name_ = "TargetAngularRates";

  n_ = static_cast<unsigned int>(Q.rows()); // Number of states
  m_ = static_cast<unsigned int>(R.rows()); // Number of measurements

  // Supported case:
  // n = 18: [x y z \psi \theta \phi
  //          \dot{x} \dot{y} \dot{z} \dot{\psi} \dot{\theta} \dot{\phi}
  //          \ddot{x} \ddot{y} \ddot{z} \ddot{\psi} \ddot{\theta} \ddot{\phi}]
  assert(n_ == 18);
  assert(m_ <= n_);
  assert(dt0>=0.0);

  A_.resize(n_, n_);
  A_.setZero();
  updateA(dt0);

  // Output matrix
  C_.resize(m_, n_);
  C_.setZero();
  C_.diagonal() = Eigen::VectorXd::Ones(m_);

  // Construct the estimator
  estimator_.reset(new LinearKalmanFilter(A_, C_, Q, R, P0));

  // Initialize the state
  x_ = Eigen::VectorXd::Zero(n_);
  pose7dToPose6d(p0,pose_internal_);

  STATE_pose(x_)  = pose_internal_;
  STATE_twist(x_) = v0;
  STATE_acc(x_)   = a0;

  estimator_->init(x_);

  updateTargetState();

  printInfo();
}

void TargetAngularRates::addMeasurement(const double& dt, const Eigen::Vector7d& meas)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt);
  updateMeasurement(meas);

  // Convert from 7d pose to 6d and
  // unwrap the angles because the estimator integrates over continuous angles
  POSE_pos(vector6d_tmp_) = POSE_pos(measured_pose_); // pos
  quaterniond_tmp_.coeffs() = POSE_quat(measured_pose_); // quat
  quaterniond_tmp_.normalize(); // Normalize the quaternion
  quatToRpy(quaterniond_tmp_,vector3d_tmp_);
  vector3d_tmp_ = unwrap(meas_rpy_internal_,vector3d_tmp_);
  POSE_rpy(vector6d_tmp_) = vector3d_tmp_;
  // Let's store the values
  meas_rpy_internal_ = vector3d_tmp_;

  std::dynamic_pointer_cast<LinearKalmanFilter>(estimator_)->update(vector6d_tmp_, A_);

  updateTargetState();
  updateTime(dt);
}

void TargetAngularRates::update(const double& dt)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt);

  std::dynamic_pointer_cast<LinearKalmanFilter>(estimator_)->update(A_);

  updateTargetState();
  updateTime(dt);
}

void TargetAngularRates::updateA(const double& dt)
{
  // A matrix using this dt
  // Discrete LTI Target motion
  A_.diagonal()     = Eigen::VectorXd::Ones(n_);
  A_.diagonal(n_/3) = Eigen::VectorXd::Ones((n_*2)/3) * dt;
  A_.diagonal((n_*2)/3) = Eigen::VectorXd::Ones(n_/3) * 0.5 * dt * dt;
}

void TargetAngularRates::updateTargetState()
{
  // Read the estimated state
  x_ = estimator_->getState();
  // Read the covariance
  P_ = estimator_->getP();
  // Create T
  T_.translation() = STATE_pos(x_);
  vector3d_tmp_    = STATE_rpy(x_); //RPY
  rpyToQuat(vector3d_tmp_,quaterniond_tmp_);
  T_.linear() = quaterniond_tmp_.toRotationMatrix();
  // Update twist
  TWIST_linear(twist_) = STATE_vel(x_);
  rotToRpy(T_.linear(),vector3d_tmp_);
  // - Convert from angular rates to angular velocities (omega)
  rpyToEarBase(vector3d_tmp_,Ear_);
  TWIST_angular(twist_) = Ear_ * STATE_rates(x_);
  // Update acceleration
  acceleration_ = STATE_acc(x_);
  // Update internal variables
  isometryToPose6d(T_,pose_internal_);
}

Eigen::Vector7d TargetAngularRates::getEstimatedPose(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);

  vector6d_tmp_ = pose_internal_ + twist_*(t1-t_) + 0.5 * acceleration_*(t1-t_)*(t1-t_);
  rpyToQuat(STATE_rpy(vector6d_tmp_),quaterniond_tmp_);
  quaterniond_tmp_.normalize(); // Normalize the quaternion
  POSE_pos(vector7d_tmp_)  = POSE_pos(vector6d_tmp_);
  POSE_quat(vector7d_tmp_) = quaterniond_tmp_.coeffs();

  return vector7d_tmp_;
}

Eigen::Vector6d TargetAngularRates::getEstimatedTwist(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);
  return twist_ + acceleration_*(t1-t_);
}
