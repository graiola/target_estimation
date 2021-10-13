#include "target_estimation/publisher_franka.h"

#define DEBUG

PublisherFranka::PublisherFranka(ros::NodeHandle& nh)
{
  // Subscribe to /tf topic where the estimaed pose of the target is published by target_node
  target_estimation_sub_ = nh.subscribe("/tf", 1, &PublisherFranka::estimationCallback, this);

  // Publish to Fraka Equilibrium Pose topic
  const std::string robot_eq_point_topic_name = "cartesian_impedance_example_controller/equilibrium_pose";
  franka_eq_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(robot_eq_point_topic_name, 1, true);
}

PublisherFranka::PublisherFranka(ros::NodeHandle& nh, const std::string &robot_eq_point_topic_name)
{
  // Subscribe to /tf topic where the estimaed pose of the target is published by target_node
  target_estimation_sub_ = nh.subscribe("/tf", 1, &PublisherFranka::estimationCallback, this);

  // Publish to Fraka Equilibrium Pose topic
  franka_eq_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(robot_eq_point_topic_name, 1, true);
}

void PublisherFranka::estimationCallback(const tf2_msgs::TFMessage::ConstPtr& estimated_pose_msg)
{
  // 1- get target name
  // 2- if target_token == string(1) && target_name(end) == "est" then:
  //    3A- if(map[target_id].count) -> update and send
  //    3B - else -> assign new value in the map, update and send
} // end callback

void PublisherFranka::setTokenName(const std::string &token_name)
{
  target_token_name_ = token_name;
}
