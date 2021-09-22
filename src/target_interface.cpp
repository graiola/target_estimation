/**
*
*/

#include "target_estimation/target_interface.hpp"

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

