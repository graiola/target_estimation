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
  initPose(pose_internal_);
  initPose(measured_pose_);
  twist_.setZero();
  acceleration_.setZero();
  t_ = t0;
  P_ = P0;


#ifdef LOGGER_ON
  std::string name = "/target_"+std::to_string(id); // Note that we publish in the ros namespace "/"
  //                                   Publisher name              Data               Data name         //
  RtLogger::getLogger().addPublisher(  name,                       measured_pose_,    "measurement"      );
  RtLogger::getLogger().addPublisher(  name,                       pose_internal_,    "pose"             );
  RtLogger::getLogger().addPublisher(  name,                       twist_,            "twist"            );
  RtLogger::getLogger().addPublisher(  name,                       acceleration_,     "acceleration"     );
  RtLogger::getLogger().addPublisher(  name,                       P_,                "covariance"       );
#endif
}

TargetInterface::~TargetInterface()
{
#ifdef LOGGER_ON
  RtLogger::getLogger().removePublishers();
#endif
}

void TargetInterface::log()
{
#ifdef LOGGER_ON
  rt_logger::RtLogger::getLogger().publish(ros::Time::now());
#endif
}

void TargetInterface::printInfo()
{
  std::cout << std::endl;
  std::cout << "*** "+this->class_name_+" ***" << std::endl;
  std::cout << "id: "<< id_ << std::endl;
  std::cout << "n:  "<< n_ << std::endl;
  std::cout << "m:  "<< m_ << std::endl;
  std::cout << std::endl;
  std::cout << "*** System Matrices ***" << std::endl;
  std::cout << "A:" << std::endl;
  std::cout << A_ << std::endl;
  std::cout << "C:" << std::endl;
  std::cout << C_ << std::endl;
  std::cout << std::endl;
  std::cout << "*** Covariance Matrices ***" << std::endl;
  std::cout << "Q:" << std::endl;
  std::cout << estimator_->getQ() << std::endl;
  std::cout << "R:" << std::endl;
  std::cout << estimator_->getR() << std::endl;
  std::cout << "P0:" << std::endl;
  std::cout << estimator_->getP0() << std::endl;
}

double TargetInterface::getPeriodEstimate()
{
  double omega_norm = TWIST_angular(twist_).norm();
  if(omega_norm>0)
    return 2*M_PI/TWIST_angular(twist_).norm();
  else
    return -1.0; // The target is not rotating
}

double TargetInterface::getTime()
{
  lock_guard<mutex> lg(data_lock_);
  return t_;
}

const Eigen::Isometry3d& TargetInterface::getEstimatedTransform() {
  lock_guard<mutex> lg(data_lock_);
  return T_;
}

const Eigen::Vector7d& TargetInterface::getEstimatedPose() {
  lock_guard<mutex> lg(data_lock_);
  isometryToPose7d(T_,vector7d_tmp_);
  return vector7d_tmp_;
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

const Eigen::Vector7d& TargetInterface::getMeasuredPose()
{
  lock_guard<mutex> lg(data_lock_);
  return measured_pose_;
}

Eigen::Vector7d TargetInterface::getEstimatedPose(const double& /*t1*/)
{
  lock_guard<mutex> lg(data_lock_);
  isometryToPose7d(T_,vector7d_tmp_);
  return vector7d_tmp_;
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

