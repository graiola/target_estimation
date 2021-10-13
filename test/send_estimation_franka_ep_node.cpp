#include "target_estimation/publisher_franka.h"

const std::string robot_eq_point_topic_name = "cartesian_impedance_example_controller/equilibrium_pose";
const std::string child_target_token = "target";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_estimation_franka_ep_node");
  ros::NodeHandle nh;

  PublisherFranka publisher(nh, robot_eq_point_topic_name);
  publisher.setTokenName(child_target_token);

  double f = 50; // Hz -> remember to use the corresponding YAML file
  double dt = 1.0/f;

  ros::Rate rate(f);

  while( ros::ok() )
  {

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
