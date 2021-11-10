#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;

RosTargetManager::RosTargetManager(ros::NodeHandle& nh):
  token_name_("target"),
  t_(0.0),
  pos_th_(0.01),
  ang_th_(0.01),
  interception_pose_on_(false),
  expiration_time_(1000.0) // Dummy value
{
  // subscribe to /tf topic
  nh_ = nh;
  meas_subscriber_ = nh_.subscribe("/tf", 1 , &RosTargetManager::measurementCallBack, this);

  initPose(interception_pose_);

  // Load the parameters for initializing the KF
  if(!parseSquareMatrix(nh_,"Q",Q_) || !parseSquareMatrix(nh_,"R",R_) || !parseSquareMatrix(nh_,"P",P_))
    throw std::runtime_error("Can not load the Cov Matrices!");

  if(!parseTargetType(nh_,type_))
    throw std::runtime_error("Can not load filter type!");
}

void RosTargetManager::setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius)
{
  manager_.setInterceptionSphere(pos,radius);
}

void RosTargetManager::measurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg)
{
  for(unsigned int i = 0; i< pose_msg->transforms.size(); i++)
  {
    const std::string& current_tf_name = pose_msg->transforms[i].child_frame_id;
    if(current_tf_name.find(token_name_) != std::string::npos)
    {

      unsigned int id;
      if(!getId(current_tf_name,id)) // If we don't have a correct id
        break;

      double current_time_stamp = 0.0;
      double prev_time_stamp = 0.0;

      meas_lock_.lock();
      if(measurements_.count(id)) // If the target has been seen before check if the measurement is new or not
      {
        current_time_stamp = toSec(pose_msg->transforms[i].header.stamp.sec,pose_msg->transforms[i].header.stamp.nsec);
        prev_time_stamp = toSec(measurements_[id].tr_.header.stamp.sec,measurements_[id].tr_.header.stamp.nsec);
        if(current_time_stamp > prev_time_stamp) // New measurement
        {
          measurements_[id].new_meas_ = true;
          measurements_[id].last_meas_time = current_time_stamp;
        }
        else
        {
          measurements_[id].new_meas_ = false; // No new measurement
        }
      }
      measurements_[id].tr_   = pose_msg->transforms[i]; // Save the measurement w.r.t observer
      meas_lock_.unlock();
    }
  }
}

void RosTargetManager::update(const double& dt)
{

  ros_t_ = ros::Time::now();

  if(meas_lock_.try_lock())
  {
    auto it = measurements_.cbegin();
    while (it != measurements_.cend())
    {
      const unsigned int& id = it->first;
      const geometry_msgs::TransformStamped& tr = it->second.tr_;
      const bool& new_meas = it->second.new_meas_;
      const double& last_meas_time = it->second.last_meas_time;

      transformStampedToPose7d(tr,tmp_vector7d_);

      if(manager_.getTarget(id)==nullptr) // Target does not exist, create it
        manager_.init(type_,id,dt,t_,Q_,R_,P_,tmp_vector7d_);

      if(new_meas)
        manager_.update(id,dt,tmp_vector7d_); // Estimate
      else
        manager_.update(id,dt); // Predict

      if(last_meas_time > 0.0 && (ros_t_.toSec() - last_meas_time) >= expiration_time_) // Remove expired target
      {
        ROS_WARN_STREAM("Timeout for target "<<id);
        it = measurements_.erase(it);
        manager_.erase(id);
      }
      else {
        ++it;
      }
    }
    meas_lock_.unlock();
  }
  else
    manager_.update(dt); // Predict with all the filters if measurements are updating

  auto target_ids = manager_.getAvailableTargets();

  // Publish the target's filtered poses
  for(unsigned int i=0;i<target_ids.size();i++)
  {
    manager_.getTargetPose(target_ids[i],tmp_vector7d_);
    pose7dToTFTransform(tmp_vector7d_,tmp_transform_);
    const std::string reference_frame = measurements_[target_ids[i]].tr_.header.frame_id;
    br_.sendTransform(tf::StampedTransform(tmp_transform_,ros_t_,reference_frame,token_name_+"_filt_"+to_string(target_ids[i])));
  }

  if(interception_pose_on_)
    manager_.getClosestInterceptionPose(t_,pos_th_,ang_th_,interception_pose_);

  t_= t_ + dt;

  manager_.log();
}

void RosTargetManager::setTargetTokenName(const string& token_name)
{
  token_name_ = token_name;
}

void RosTargetManager::setPositionConvergenceThreshold(const double& th)
{
  assert(th>=0.0);
  pos_th_ = th;
}

void RosTargetManager::setAngularConvergenceThreshold(const double& th)
{
  assert(th>=0.0);
  ang_th_ = th;
}

const Eigen::Vector7d& RosTargetManager::getInterceptionPose() const
{
  return interception_pose_;
}

void RosTargetManager::calculateInterceptionPose(bool active)
{
  interception_pose_on_ = active;
}

void RosTargetManager::setExpirationTime(double time)
{
  assert(time>=0.0);
  expiration_time_ = time;
}

bool RosTargetManager::parseSquareMatrix(const ros::NodeHandle& n, const std::string& matrix, Eigen::MatrixXd& M)
{
  std::vector<double> Mv;
  if (n.getParam(matrix, Mv))
  {
    unsigned int size = static_cast<unsigned int>(std::sqrt(Mv.size()));
    M = Eigen::Map<Eigen::MatrixXd>(Mv.data(),size,size);
  }
  else
  {
    ROS_ERROR_STREAM("Can not find matrix: "<<matrix);
    return false;
  }

  return true;
}

bool RosTargetManager::parseTargetType(const ros::NodeHandle& n, TargetManager::target_t& type)
{
  std::string type_str;
  if (n.getParam("type", type_str))
  {
    if(manager_.selectTargetType(type_str,type))
      return true;
    else
      return false;
  }
  else
  {
    ROS_ERROR_STREAM("Can not find type");
    return false;
  }
}
