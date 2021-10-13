#ifndef PUBLISHER_FRANKA_H
#define PUBLISHER_FRANKA_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/convert.h>

#include <Eigen/Core>
#include <vector>
#include <mutex>

#include "target_estimation/utils.hpp"
#include "target_estimation/target_manager_ros.hpp"

class PublisherFranka
{
private:
  // --- Priavte Definitions and Typedefs --- //

public:
  PublisherFranka(ros::NodeHandle& nh);
  PublisherFranka(ros::NodeHandle& nh, const std::string &robot_eq_point_topic_name);

  void setTokenName(const std::string &token_name);
private:

  // --- Priavte Functions --- //
  void estimationCallback(const tf2_msgs::TFMessage::ConstPtr& estimated_pose_msg);

  // --- Priavte Variables --- //
  std::mutex meas_lock_;

  std::string target_token_name_ = "";

  // --- ROS MSGs --- //
  geometry_msgs::PoseStamped franka_eq_pose_msg_;

  // --- ROS Publishers and Subscribers --- //
  ros::Subscriber target_estimation_sub_;
  ros::Publisher franka_eq_pose_pub_;
};

#endif // PUBLISHER_FRANKA_H
