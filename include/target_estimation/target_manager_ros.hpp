#ifndef TARGET_MANAGER_ROS_HPP
#define TARGET_MANAGER_ROS_HPP

#include <iostream>
#include <iomanip>
#include <vector>

#include <target_estimation/TargetEstimation.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <rt_logger/rt_logger.h>
#include <atomic>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include "target_estimation/target_manager.hpp"

class RosTargetManager
{

public:

  RosTargetManager(ros::NodeHandle& nh);

  void setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius);

  void update(const double& dt);

  void setTargetTokenName(const std::string& token_name);

  void setReferenceFrameName(const std::string& frame);

  void setCameraFrameName(const std::string& frame);

  void setCameraTransform(const Eigen::Isometry3d& reference_T_camera);

private:

  struct Measurement
  {
    geometry_msgs::TransformStamped tr_;
    bool new_meas_ = true;
  };

  typedef std::map<unsigned int, Measurement> meas_map_t;

  void measurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg);
  bool parseSquareMatrix(const ros::NodeHandle& n, const std::string& matrix, Eigen::MatrixXd& M);
  bool parseTargetType(const ros::NodeHandle& n, TargetManager::target_t& type);

  ros::NodeHandle nh_;
  ros::Subscriber meas_subscriber_;

  TargetManager manager_;
  TargetManager::target_t type_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd R_;

  std::string token_name_;
  std::string reference_frame_;
  std::string camera_frame_;

  Eigen::Isometry3d reference_T_camera_;

  double t_;
  double t_prev_;
  double dt_;

  std::mutex meas_lock_;

  meas_map_t measurements_;

  Eigen::Vector7d tmp_vector7d_;
  Eigen::Isometry3d tmp_isometry3d_;

  tf::TransformBroadcaster br_;
};

#endif
