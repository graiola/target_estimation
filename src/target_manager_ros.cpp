#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;

RosTargetManager::RosTargetManager(ros::NodeHandle& nh):
  t_(0.0),
  expiration_time_(1000.0), // Dummy value
  new_reference_frame_(""),
  predict_(true)
{
  // subscribe to /tf topic
  nh_ = nh;
  meas_subscriber_ = nh_.subscribe("/tf", 1 , &RosTargetManager::measurementCallBack, this);

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
    for(unsigned int i=0;i<token_names_.size();i++)
        if(current_tf_name.find(token_names_[i]) != std::string::npos)
        {
          unsigned int id;
          if(!getId(current_tf_name,id)) // If we don't have a correct id
            break;
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
    int id = it->first;
    double last_meas_time = it->second.getTime();
    if(it->second.read(tmp_tr_))
    {
      transformStampedToPose7d(tmp_tr_,tmp_vector7d_);
      if(manager_.getTarget(id)==nullptr) // Target does not exist, create it
        manager_.init(type_,id,dt,t_,Q_,R_,P_,tmp_vector7d_);
      manager_.update(id,dt,tmp_vector7d_); // Estimate
    }
    else if(predict_)
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

  auto target_ids = manager_.getAvailableTargets();

  // Publish the target's filtered poses
  for(unsigned int i=0;i<target_ids.size();i++)
  {
    manager_.getTargetPose(target_ids[i],tmp_vector7d_);
    pose7dToTFTransform(tmp_vector7d_,tmp_tf_tr_);
    const std::string& reference_frame = measurements_[target_ids[i]].getFrameId();
    const std::string& current_frame   = measurements_[target_ids[i]].getChildFrameId();
    if(!new_reference_frame_.empty())
    {
        tf_listener_.lookupTransform(new_reference_frame_,reference_frame,ros_t_,tmp_tf_stamped_tr_); // new_T_old
        tmp_tf_tr_ = tmp_tf_stamped_tr_ * tmp_tf_tr_; // new_T_target = new_T_old * old_T_target
        tf_broadcaster_.sendTransform(tf::StampedTransform(tmp_tf_tr_,ros_t_,new_reference_frame_,current_frame+"_filt"));
    }
    else
        tf_broadcaster_.sendTransform(tf::StampedTransform(tmp_tf_tr_,ros_t_,reference_frame,current_frame+"_filt"));
  }

  t_= t_ + dt;

  manager_.log();
}

void RosTargetManager::setTargetTokenNames(const std::vector<std::string>& token_names)
{
  token_names_ = token_names;
}

void RosTargetManager::setExpirationTime(double time)
{
  assert(time>=0.0);
  expiration_time_ = time;
}

void RosTargetManager::setNewReferenceFrame(const std::string& frame_name)
{
    new_reference_frame_ = frame_name;
}

void RosTargetManager::activatePrediction(bool predict)
{
    predict_ = predict;
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
