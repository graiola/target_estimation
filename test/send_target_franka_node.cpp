#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include "geometry_msgs/PoseStamped.h"

void estimationCallback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_target_franka_node");
  ros::NodeHandle nh;

  // Subscribe to /tf topic
  ros::Subscriber pose_sub = nh.subscribe("/tf", 1, estimationCallback);

  // Publish to Fraka Equilibrium Pose topic
  std::string topic_to_publish = "cartesian_impedance_example_controller/equilibrium_pose";
  ros::Publisher franka_eq_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(topic_to_publish, 1, true);

  ros::spin();

  return 0;
}

void estimationCallback(const std_msgs::String::ConstPtr& msg)
{

}
