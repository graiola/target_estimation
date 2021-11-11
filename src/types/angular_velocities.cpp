/**
*
*/

#include "target_estimation/types/angular_velocities.hpp"

using namespace std;

// Helpers
#define STATE_twist(x)           x.segment(6,6)
#define STATE_vel(x)             x.segment(6,3)
#define STATE_omega(x)           x.segment(9,3)
#define STATE_pos(x)             x.segment(0,3)
#define STATE_pose(x)            x.segment(0,6)
#define STATE_rpy(x)             x.segment(3,3)
#define STATE_acc(x)             x.segment(12,6)

// ----------------------------
// TargetAngularVelocities
// ----------------------------
TargetAngularVelocities::TargetAngularVelocities(const unsigned int& id,
                                                 const double& dt0,
                                                 const double& t0,
                                                 const Eigen::MatrixXd&   Q,
                                                 const Eigen::MatrixXd&   R,
                                                 const Eigen::MatrixXd&   P0,
                                                 const Eigen::Vector7d&   p0,
                                                 const Eigen::Vector6d&   v0,
                                                 const Eigen::Vector6d&   /*a0*/)
  : TargetInterface(id,P0,t0)
{
  class_name_ = "TargetAngularVelocities";

  n_ = static_cast<unsigned int>(Q.rows()); // Number of states
  m_ = static_cast<unsigned int>(R.rows()); // Number of measurements

  // Supported case:
  // n = 12: [x y z \psi \theta \phi
  //          \dot{x} \dot{y} \dot{z} omega_x omega_y omega_z]
  assert(n_ == 12);
  assert(m_ <= n_);
  assert(dt0>=0.0);

  // Identity matrix
  I3_.setIdentity();

  // Resize tmp vector
  vectorXd_tmp_.resize(n_);
  vectorXd_tmp_.setZero();

  // Initialize the state
  x_ = Eigen::VectorXd::Zero(n_);
  pose7dToPose6d(p0,pose_internal_);

  STATE_pose(x_)  = pose_internal_;
  STATE_twist(x_) = v0;

  // Linearized transition matrix
  A_.resize(n_, n_);
  A_.setZero();
  updateA(dt0,STATE_rpy(x_),STATE_omega(x_));

  // Linearized output matrix
  C_.resize(m_, n_);
  C_.setZero();
  C_.diagonal() = Eigen::VectorXd::Ones(m_);

  // Construct the estimator
  estimator_.reset(new ExtendedKalmanFilter(std::bind(&TargetAngularVelocities::f,this,std::placeholders::_1,dt0),
                                            std::bind(&TargetAngularVelocities::h,this,std::placeholders::_1),
                                            A_, C_, Q, R, P0));

  estimator_->init(x_);

  updateTargetState();

  printInfo();
}

void TargetAngularVelocities::addMeasurement(const double& dt, const Eigen::Vector7d& meas)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt,STATE_rpy(x_),STATE_omega(x_));

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

  std::dynamic_pointer_cast<ExtendedKalmanFilter>(estimator_)->update(vector6d_tmp_,std::bind(&TargetAngularVelocities::f,this,std::placeholders::_1,dt),A_);

  updateTargetState();
  updateTime(dt);
  updateMeasurement(meas);
}

void TargetAngularVelocities::update(const double& dt)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt,STATE_rpy(x_),STATE_omega(x_));

  std::dynamic_pointer_cast<ExtendedKalmanFilter>(estimator_)->update(std::bind(&TargetAngularVelocities::f,this,std::placeholders::_1,dt),A_);

  updateTargetState();
  updateTime(dt);
}

void TargetAngularVelocities::updateA(const double& dt, const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega)
{
  A_.block<3,3>(0,0)   = I3_;
  A_.block<3,3>(0,6)   = I3_ * dt;
  A_.block<3,3>(3,3)   = EarBaseInvJacobianRpy(rpy,omega,dt);
  A_.block<3,3>(3,9)   = EarBaseInvJacobianOmega(rpy,dt);
  A_.block<3,3>(6,6)   = I3_;
  A_.block<3,3>(9,9)   = I3_;
}

Eigen::VectorXd TargetAngularVelocities::f(const Eigen::VectorXd& x, const double& dt)
{
  assert(x.size() == n_);

  vectorXd_tmp_.setZero();

  rpyToEarBaseInv(STATE_rpy(x),EarInv_);

  STATE_pos(vectorXd_tmp_)    = STATE_pos(x) + dt * STATE_vel(x);
  STATE_vel(vectorXd_tmp_)    = STATE_vel(x);
  STATE_omega(vectorXd_tmp_)  = STATE_omega(x);
  STATE_rpy(vectorXd_tmp_)    = STATE_rpy(x) + dt * EarInv_ * STATE_omega(x);

  return vectorXd_tmp_;
}

Eigen::VectorXd TargetAngularVelocities::h(const Eigen::VectorXd& x)
{
  assert(x.size() == n_);

  y_.setZero();
  POSE_pos(y_)  = STATE_pos(x);
  POSE_rpy(y_) = STATE_rpy(x);

  return y_;
}

void TargetAngularVelocities::updateTargetState()
{
  // Read the estimated state
  x_ = estimator_->getState();
  // Read the covariance
  P_ = estimator_->getP();
  // Set our Target's state
  position_ = STATE_pos(x_);
  POSE_pos(pose_) = position_;
  pose_internal_ = STATE_pose(x_);
  rpy_ = STATE_rpy(x_);
  rpyToQuat(rpy_,q_);
  POSE_quat(pose_) = q_.coeffs();
  T_.translation() = position_;
  T_.linear() = q_.toRotationMatrix();
  TWIST_linear(twist_) = STATE_vel(x_);
  TWIST_angular(twist_) = STATE_omega(x_);
}

Eigen::Vector7d TargetAngularVelocities::getEstimatedPose(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);

  //vector6d_tmp_ = pose_internal_ + twist_*(t1-t_);
  //rpyToQuat(STATE_rpy(vector6d_tmp_),quaterniond_tmp_);
  //quaterniond_tmp_.normalize(); // Normalize the quaternion
  //POSE_pos(vector7d_tmp_)  = POSE_pos(vector6d_tmp_);
  //POSE_quat(vector7d_tmp_) = quaterniond_tmp_.coeffs();

  vector3d_tmp_.setZero();
  vector3d_tmp_ = position_ + TWIST_linear(twist_) * (t1-t_);
  rpyToQuat(POSE_rpy(pose_internal_),quaterniond_tmp_);
  quaterniond_tmp_.coeffs() = Qtran(t1-t_,TWIST_angular(twist_)) * quaterniond_tmp_.coeffs();
  quaterniond_tmp_.normalize(); // Normalize the quaternion
  POSE_pos(vector7d_tmp_)  = vector3d_tmp_;
  POSE_quat(vector7d_tmp_) = quaterniond_tmp_.coeffs();

  return vector7d_tmp_;
}

