/**
*
*/

#include "target_estimation/target.hpp"

#ifdef LOGGER_ON
#include <rt_logger/rt_logger.h>
using namespace rt_logger;
#endif

using namespace std;


// ----------------------------
// TargetInterface
// ----------------------------
TargetInterface::TargetInterface(const unsigned int& id, const Eigen::MatrixXd P0, const double& t0)
{
  assert(t0>=0);
  id_ = static_cast<int>(id);
  n_meas_ = 0;
  T_.setIdentity();
  initPose(pose_);
  initPose(intersection_pose_);
  initPose(measured_pose_);
  position_.setZero();
  rpy_.setZero();
  q_.setIdentity();
  twist_.setZero();
  acceleration_.setZero();
  acceleration_on_ = false;
  t_ = t0;
  P_ = P0;
  pos_error_filter_.reset(new MovingAvgFilter(250)); // with 50hz loop we have 5.0 secs of filter
  ang_error_filter_.reset(new MovingAvgFilter(250)); // with 50hz loop we have 5.0 secs of filter

#ifdef LOGGER_ON
  std::string name = "/target_"+std::to_string(id); // Note that we publish in the ros namespace "/"
  //                                   Publisher name              Data               Data name         //
  RtLogger::getLogger().addPublisher(  name,                       measured_pose_,    "measurement"      );
  RtLogger::getLogger().addPublisher(  name,                       pose_,             "pose"             );
  RtLogger::getLogger().addPublisher(  name,                       intersection_pose_,"intersection_pose");
  RtLogger::getLogger().addPublisher(  name,                       twist_,            "twist"            );
  RtLogger::getLogger().addPublisher(  name,                       acceleration_,     "acceleration"     );
  RtLogger::getLogger().addPublisher(  name,                       position_,         "position"         );
  RtLogger::getLogger().addPublisher(  name,                       rpy_,              "rpy"              );
  RtLogger::getLogger().addPublisher(  name,                       P_,                "covariance"       );
#endif
}

void TargetInterface::log()
{
#ifdef LOGGER_ON
  rt_logger::RtLogger::getLogger().publish(ros::Time::now());
#endif
}

double TargetInterface::getPeriodEstimate()
{
  double omega_norm = TWIST_angular(twist_).norm();
  if(omega_norm>0)
    return 2*M_PI/TWIST_angular(twist_).norm();
  else
    return -1.0; // The target is not rotating
}

double TargetInterface::getIntersectionTime(const double& t1, const Eigen::Vector3d& origin, const double& radius)
{

  pos_tmp_ = getEstimatedPose(t1).head(3);
  vel_tmp_ = getEstimatedTwist(t1).head(3);
  acc_tmp_ = getEstimatedAcceleration(t1).head(3);

  // Use only the translational part of the variables
  double x = pos_tmp_(0) - origin(0);
  double y = pos_tmp_(1) - origin(1);
  double z = pos_tmp_(2) - origin(2);
  double vx = vel_tmp_(0);
  double vy = vel_tmp_(1);
  double vz = vel_tmp_(2);
  double ax = acc_tmp_(0);
  double ay = acc_tmp_(1);
  double az = acc_tmp_(2);
  double R = radius;

  // Polynomial coefficients
  Eigen::VectorXd coeff;

  if(acceleration_on_)
  {
    // 2rd order system
    coeff.resize(5);
    coeff(4) = 0.25 * ax*ax + ay*ay + az*az;
    coeff(3) = vx*ax + vy*ay + vz*az;
    coeff(2) = vx*vx + vy*vy + vz*vz + x*ax + y*ay + z*az;
    coeff(1) = 2*(x*vx + y*vy + z*vz);
    coeff(0) = x*x + y*y + z*z - R*R;
  }
  else
  {
    // 1st order system
    coeff.resize(3);
    coeff(2) = vx*vx + vy*vy + vz*vz;
    coeff(1) = 2*(x*vx + y*vy + z*vz);
    coeff(0) = x*x + y*y + z*z - R*R;
  }

  double delta_intersect_t = solver_.lowestRealRoot(coeff);

  if(delta_intersect_t < 0) return -1;

  return delta_intersect_t;
}

bool TargetInterface::getIntersectionPose(const double& t1, const double& pos_th, const double& ang_th, const Eigen::Vector3d& origin, const double& radius, Eigen::Vector7d& intersection_pose)
{
  assert(t1>=0.0);
  assert(pos_th>=0.0);
  assert(ang_th>=0.0);
  double delta_intersect_t = -1;
  bool converged = false;
  initPose(intersection_pose);
  delta_intersect_t = getIntersectionTime(t1,origin,radius);
  if (delta_intersect_t > -1)
  {
    Eigen::Quaterniond q1, q2;
    intersection_pose = getEstimatedPose(delta_intersect_t+t1);
    double pos_error = (POSE_pos(intersection_pose) - POSE_pos(intersection_pose_)).norm();
    q1.coeffs() = POSE_quat(intersection_pose);
    q2.coeffs() = POSE_quat(intersection_pose_);
    q1.normalize();
    q2.normalize();
    double ang_error = std::abs(wrapMinMax(computeQuaternionErrorAngle(q1,q2), -M_PI, M_PI));

    // Check if errors converged  using a moving average filter
    double pos_error_filt = pos_error_filter_->update(pos_error);
    double ang_error_filt = ang_error_filter_->update(ang_error);

    // Save the value
    intersection_pose_ = intersection_pose;

    if(pos_error_filt <= pos_th && ang_error_filt <= ang_th)
      converged = true;

  }
  return converged;
}

double TargetInterface::getTime()
{
  lock_guard<mutex> lg(data_lock_);
  return t_;
}

const Eigen::Vector7d& TargetInterface::getEstimatedPose() {
  lock_guard<mutex> lg(data_lock_);
  return pose_;
}

const Eigen::Isometry3d& TargetInterface::getEstimatedTransform() {
  lock_guard<mutex> lg(data_lock_);
  return T_;
}

const Eigen::Vector3d& TargetInterface::getEstimatedPosition() {
  lock_guard<mutex> lg(data_lock_);
  return position_;
}

const Eigen::Quaterniond& TargetInterface::getEstimatedOrientation() {
  lock_guard<mutex> lg(data_lock_);
  return q_;
}

const Eigen::Vector3d& TargetInterface::getEstimatedRPY() {
  lock_guard<mutex> lg(data_lock_);
  return rpy_;
}

const Eigen::Vector6d& TargetInterface::getEstimatedTwist() {
  lock_guard<mutex> lg(data_lock_);
  return twist_;
}

const Eigen::Vector6d& TargetInterface::getEstimatedAcceleration()
{
  lock_guard<mutex> lg(data_lock_);
  return acceleration_;
}

const Eigen::Vector7d& TargetInterface::getIntersectionPose()
{
  lock_guard<mutex> lg(data_lock_);
  return intersection_pose_;
}

const Eigen::Vector7d& TargetInterface::getMeasuredPose()
{
  lock_guard<mutex> lg(data_lock_);
  return measured_pose_;
}

Eigen::Vector7d TargetInterface::getEstimatedPose(const double& /*t1*/)
{
  lock_guard<mutex> lg(data_lock_);
  return pose_;
}

Eigen::Vector6d TargetInterface::getEstimatedTwist(const double& /*t1*/)
{
  lock_guard<mutex> lg(data_lock_);
  return twist_;
}

Eigen::Vector6d TargetInterface::getEstimatedAcceleration(const double& /*t1*/)
{
  lock_guard<mutex> lg(data_lock_);
  return acceleration_;
}

void TargetInterface::updateMeasurement(const Eigen::Vector7d& meas)
{
  measured_pose_ = meas;
  n_meas_ += 1;
}

void TargetInterface::updateTime(const double& dt)
{
  assert(dt>=0.0);
  t_ = t_ + dt;
}

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
  std::cout<< "n: "<< n_ << std::endl;
  std::cout<< "m: "<< m_ << std::endl;
  std::cout<< "dt0: "<< dt0 << std::endl;
  std::cout<< "t0: "<< t0 << std::endl;
  std::cout<< "x0:" << std::endl;
  std::cout<< x_.transpose() << std::endl;
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

// ----------------------------
// TargetRPYExtended
// ----------------------------
TargetRPYExtended::TargetRPYExtended(const unsigned int& id,
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
  //          \dot{x} \dot{y} \dot{z} omega_x omega_y omega_z]
  assert(n_ == 12);
  assert(m_ <= n_);
  assert(dt0>=0.0);

  // Useful identity matrices
  I3_.setIdentity();

  // Resize tmp vector
  vectorXd_tmp_.resize(n_);
  vectorXd_tmp_.setZero();

  // Initialize the state
  x_ = Eigen::VectorXd::Zero(n_);

  // Initialize the state
  x_ = Eigen::VectorXd::Zero(n_);
  pose7d2pose6d(p0,pose_internal_);

  RPY_TARGET_pose(x_)  = pose_internal_;

  // Linearized transition matrix
  A_.resize(n_, n_);
  updateA(dt0,RPY_TARGET_rpy(x_),RPY_TARGET_omega(x_));

  // Linearized output matrix
  C_.resize(m_, n_);
  C_.setZero();
  C_.diagonal() = Eigen::VectorXd::Ones(m_);

  std::cout << std::endl;
  std::cout << "*** TargetRPYExtended ***" << std::endl;
  std::cout << "*** Initialization ***" << std::endl;
  std::cout<< "n: "<< n_ << std::endl;
  std::cout<< "m: "<< m_ << std::endl;
  std::cout<< "dt0: "<< dt0 << std::endl;
  std::cout<< "t0: "<< t0 << std::endl;
  std::cout<< "x0:" << std::endl;
  std::cout<< x_.transpose() << std::endl;
  std::cout << std::endl;
  std::cout << "*** System Matrices ***" << std::endl;
  std::cout << "A (linearized):" << std::endl;
  std::cout << A_ << std::endl;
  std::cout << "C (linearized):" << std::endl;
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

  // Construct the estimator
  estimator_.reset(new ExtendedKalmanFilter(std::bind(&TargetRPYExtended::f,this,std::placeholders::_1,dt0),
                                            std::bind(&TargetRPYExtended::h,this,std::placeholders::_1),
                                            A_, C_, Q, R, P0));

  estimator_->init(x_);

  updateTargetState();
}

void TargetRPYExtended::addMeasurement(const double& dt, const Eigen::Vector7d& meas)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt,RPY_TARGET_rpy(x_),RPY_TARGET_omega(x_));

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

  std::dynamic_pointer_cast<ExtendedKalmanFilter>(estimator_)->update(vector6d_tmp_,std::bind(&TargetRPYExtended::f,this,std::placeholders::_1,dt),A_);

  updateTargetState();
  updateTime(dt);
  updateMeasurement(meas);
}

void TargetRPYExtended::update(const double& dt)
{
  lock_guard<mutex> lg(data_lock_);

  updateA(dt,RPY_TARGET_rpy(x_),RPY_TARGET_omega(x_));

  std::dynamic_pointer_cast<ExtendedKalmanFilter>(estimator_)->update(std::bind(&TargetRPYExtended::f,this,std::placeholders::_1,dt),A_);

  updateTargetState();
  updateTime(dt);
}

void TargetRPYExtended::updateA(const double& dt, const Eigen::Vector3d& rpy, const Eigen::Vector3d& omega)
{
  A_.block<3,3>(0,0)   = I3_;
  A_.block<3,3>(0,6)   = I3_ * dt;;
  A_.block<3,3>(3,3)   = EarBaseInvJacobianRpy(rpy,omega,dt);
  A_.block<3,3>(3,9)   = EarBaseInvJacobianOmega(rpy,dt);
  A_.block<3,3>(6,6)   = I3_;
  A_.block<3,3>(9,9)   = I3_;
}

Eigen::VectorXd TargetRPYExtended::f(const Eigen::VectorXd& x, const double& dt)
{
  assert(x.size() == n_);

  vectorXd_tmp_.setZero();

  rpyToEarBaseInv(RPY_TARGET_rpy(x),EarInv_);

  RPY_TARGET_pos(vectorXd_tmp_)    = RPY_TARGET_pos(x) + dt * RPY_TARGET_vel(x);
  RPY_TARGET_vel(vectorXd_tmp_)    = RPY_TARGET_vel(x);
  RPY_TARGET_omega(vectorXd_tmp_)  = RPY_TARGET_omega(x);
  RPY_TARGET_rpy(vectorXd_tmp_)    = RPY_TARGET_rpy(x) + dt * EarInv_ * RPY_TARGET_omega(x);

  return vectorXd_tmp_;
}

Eigen::VectorXd TargetRPYExtended::h(const Eigen::VectorXd& x)
{
  assert(x.size() == n_);

  y_.setZero();
  POSE_pos(y_)  = RPY_TARGET_pos(x);
  POSE_rpy(y_) = RPY_TARGET_rpy(x);

  return y_;
}

void TargetRPYExtended::updateTargetState()
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
  TWIST_angular(twist_) = RPY_TARGET_omega(x_);
}

Eigen::Vector7d TargetRPYExtended::getEstimatedPose(const double& t1)
{
  lock_guard<mutex> lg(data_lock_);

  //vector6d_tmp_ = pose_internal_ + twist_*(t1-t_);
  //rpyToQuat(RPY_TARGET_rpy(vector6d_tmp_),quaterniond_tmp_);
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

