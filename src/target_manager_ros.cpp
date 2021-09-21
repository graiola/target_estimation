#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;
using namespace rt_logger;

void transformStampedToEigen7d(const geometry_msgs::TransformStamped& t, Eigen::Vector7d& e)
{
  e(0) = t.transform.translation.x;
  e(1) = t.transform.translation.y;
  e(2) = t.transform.translation.z;

  e(3) = t.transform.rotation.x;
  e(4) = t.transform.rotation.y;
  e(5) = t.transform.rotation.z;
  e(6) = t.transform.rotation.w;
}

void eigen7dToTfTransform(const Eigen::Vector7d& e, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(e(0),e(1),e(2)));
  t.setRotation(tf::Quaternion(e(3),e(4),e(5),e(6)));
}

RosTargetManager::RosTargetManager(ros::NodeHandle& nh):
  token_name_(""),
  reference_frame_("")
{
  // subscribe to /tf topic
  nh_ = nh;
  meas_subscriber_ = nh_.subscribe("/tf", 1 , &RosTargetManager::measurementCallBack, this); // FIXME hardcoded

  t_ = 0.0;
  dt_ = 0.01;
  t_prev_ = 0.0;

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

      if(measurements_.count(id)) // If the target has been seen before check if the measurement is new or not
      {
        double current_time_stamp = toSec(pose_msg->transforms[i].header.stamp.sec,pose_msg->transforms[i].header.stamp.nsec);
        double prev_time_stamp = toSec(measurements_[id].tr_.header.stamp.sec,measurements_[id].tr_.header.stamp.nsec);

        if(current_time_stamp > prev_time_stamp)
          measurements_[id].new_meas_ = true; // New measurement
        else
          measurements_[id].new_meas_ = false; // No new measurement
      }

      meas_lock_.lock();
      measurements_[id].tr_ = pose_msg->transforms[i]; // Save the measurement
      meas_lock_.unlock();

    }
  }
}

void RosTargetManager::update(const double& dt)
{

  double t = ros::Time::now().toSec();
  dt_ = t - t_prev_;

  if(meas_lock_.try_lock())
  {
    for(const auto& tmp : measurements_)
    {
      const unsigned int& id = tmp.first;
      const geometry_msgs::TransformStamped& tr = tmp.second.tr_;
      const bool& new_meas = tmp.second.new_meas_;

      transformStampedToEigen7d(tr,tmp_vector7d_);

      if(manager_.getTarget(id)==nullptr) // Target does not exist, create it
        manager_.init(id,dt_,Q_,R_,P_,tmp_vector7d_,0.0,type_);

      if(new_meas)
        manager_.update(tmp.first,dt_,tmp_vector7d_); // Estimate
      else
        manager_.update(tmp.first,dt_); // Predict
    }
    meas_lock_.unlock();
  }


  auto target_ids = manager_.getAvailableTargets(); // FIXME prb not thread safe

  for(unsigned int i=0;i<target_ids.size();i++)
  {
    Eigen::Vector7d pose;
    tf::Transform transform;
    manager_.getTargetPose(target_ids[i],pose);
    eigen7dToTfTransform(pose,transform);
    br_.sendTransform(tf::StampedTransform(transform,ros::Time::now(),reference_frame_,token_name_+"_filt_"+to_string(target_ids[i])));
  }

  t_= t_ + dt;
  t_prev_ = t;
}

void RosTargetManager::setTargetTokenName(const string &token_name)
{
  token_name_ = token_name;
}

void RosTargetManager::setReferenceFrame(const string& reference_frame)
{
  reference_frame_ = reference_frame;
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
    if (std::strcmp(type_str.c_str(),"rpy")==0)
      type = TargetManager::target_t::RPY;
    else if (std::strcmp(type_str.c_str(),"rpy_ext") == 0)
      type = TargetManager::target_t::RPY_EXT;

    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Can not find type");
    return false;
  }

}
