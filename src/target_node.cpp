#include "target_estimation/target_manager_ros.hpp"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_node");
  ros::NodeHandle nh_priv("~");

  std::string ns;
  if(!nh_priv.getParam("ns",ns))
      ns = "target_estimation";

  ros::NodeHandle nh(ns);

  // Args
  std::string token = nh_priv.param<std::string>("token", "target");
  double timeout = nh_priv.param<double>("timeout", 10.0);

  // Params
  double freq;
  if(nh.getParam("frequency",freq))
    ROS_INFO_STREAM("Target node loop frequency set to "<<freq << " Hz");
  else
    throw std::runtime_error("Loop frequency not set!");


  RosTargetManager ros_target_manager(nh);
  ros_target_manager.setTargetTokenName(token);
  ros_target_manager.setExpirationTime(timeout);

  double dt = 1.0/freq;

  ros::Rate rate(freq);

  while (ros::ok())
  {

    ros_target_manager.update(dt);

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
