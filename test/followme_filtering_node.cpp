#include "target_estimation/target_manager_ros.hpp"

#define WORLD_FRAME  "camera_depth_optical_frame"
#define CAMERA_FRAME "camera_depth_optical_frame"
#define TARGET_NAME  "target"

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "followme_filtering_node");
  ros::NodeHandle nh;

  Eigen::Vector7d interception_pose;
  initPose(interception_pose);

  Eigen::Isometry3d world_T_camera = Eigen::Isometry3d::Identity();


  RosTargetManager ros_manager(nh);
  ros_manager.setTargetTokenName(TARGET_NAME);
  ros_manager.setReferenceFrameName(WORLD_FRAME);
  ros_manager.setCameraFrameName(CAMERA_FRAME);
  ros_manager.setCameraTransform(world_T_camera);


  double f = 1000; // Hz -> remember to use the corresponding YAML file
  double dt = 1.0/f;

  ros::Rate rate(f);

  while (ros::ok())
  {

    ros_manager.update(dt);

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
