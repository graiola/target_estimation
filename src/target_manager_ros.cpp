#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;

RosTargetManager::RosTargetManager(ros::NodeHandle& nh):
  TargetManager(),
  token_name_("target"),
  t_(0.0),
  expiration_time_(1000.0) // Dummy value
{
  // subscribe to /tf topic
  nh_ = nh;
  std::string meas_topic_name{"/tf"};
  meas_subscriber_ = nh_.subscribe(meas_topic_name, 1 , &RosTargetManager::measurementCallBack, this);

  // Load the parameters for initializing the KF
  if(!parseSquareMatrix(nh_,"Q",Q_) || !parseSquareMatrix(nh_,"R",R_) || !parseSquareMatrix(nh_,"P",P_))
    throw std::runtime_error("Can not load the Cov Matrices!");

  if(!parseTargetType(nh_,type_))
    throw std::runtime_error("Can not load filter type!");
}

void RosTargetManager::measurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg)
{
  for(unsigned int i = 0; i< pose_msg->transforms.size(); i++)
  {
    const std::string& current_tf_name = pose_msg->transforms[i].child_frame_id;

    // Add second comparison in order not to consider the filetered pose as a measurement
    if(current_tf_name.find(token_name_) != std::string::npos && current_tf_name.find("filt") == std::string::npos )
    {
      unsigned int id;

      if(use_token_name_with_uint_id_)
      {
        if(!getId(current_tf_name,id)) // we don't have a correct id (target_name = token_id)
          break;
      }
      else
      {
        id = 0;
      }

      measurements_[id].update(pose_msg->transforms[i]);
    }
  }
}

void RosTargetManager::update(const double& dt)
{

  ros_t_ = ros::Time::now();

  auto it = measurements_.begin();
  while (it != measurements_.end())
  {
    unsigned int id = it->first;
    double last_meas_time = it->second.getTime();
    if(it->second.read(tmp_tr_))
    {
      transformStampedToPose7d(tmp_tr_,tmp_vector7d_);
      if(getTarget(id)==nullptr)
      {
        // Target does not exist, create it
        init(type_,id,dt,t_,Q_,R_,P_,tmp_vector7d_);
      }
      TargetManager::update(id,dt,tmp_vector7d_); // Estimate

    }
    else
    {
      TargetManager::update(id,dt); // Predict
    }

    if(last_meas_time > 0.0 && (ros_t_.toSec() - last_meas_time) >= expiration_time_) // Remove expired target
    {
      ROS_WARN_STREAM("Timeout for target "<<id);
      it = measurements_.erase(it);
      erase(id);
    }
    else {
      ++it;
    }
  }

  auto target_ids = getAvailableTargets();

  // Publish the target's filtered poses
  for(unsigned int i=0;i<target_ids.size();i++)
  {
    getTargetPose( target_ids[i], tmp_vector7d_);
    pose7dToTFTransform(tmp_vector7d_,tmp_tf_tr_);
    const std::string reference_frame = measurements_[target_ids[i]].getFrameId();
    br_.sendTransform(tf::StampedTransform(tmp_tf_tr_,ros_t_,reference_frame,token_name_+"_filt_"+to_string(target_ids[i])));
  }

  t_= t_ + dt;

  log();
}

void RosTargetManager::setTargetTokenName(const string& token_name, const bool& token_with_uint_id)
{
  token_name_ = token_name;
  use_token_name_with_uint_id_ = token_with_uint_id;
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
    if( selectTargetType(type_str,type) )
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
