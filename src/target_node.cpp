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
  std::vector<std::string> token_names;
  nh_priv.param("token", token_names, std::vector<std::string>());
  if(token_names.size() == 0)
      throw std::runtime_error("No token list provided!");

  double timeout = nh_priv.param<double>("timeout", 10.0);
  std::string new_reference_frame  = nh_priv.param<std::string>("new_reference_frame", "");
  bool activate_prediction  = nh_priv.param<bool>("activate_prediction", true);

  // Params
  double freq;
  if(nh.getParam("frequency",freq))
    ROS_INFO_STREAM("Target node loop frequency set to "<<freq << " Hz");
  else
    throw std::runtime_error("Loop frequency not set!");


  RosTargetManager ros_target_manager(nh);
  ros_target_manager.setTargetTokenNames(token_names);
  ros_target_manager.setExpirationTime(timeout);
  ros_target_manager.setNewReferenceFrame(new_reference_frame);
  ros_target_manager.activatePrediction(activate_prediction);

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
