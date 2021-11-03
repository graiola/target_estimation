#include "target_estimation/target_manager_ros.hpp"

#define WORLD_FRAME  "world"
#define CAMERA_FRAME "camera_depth_optical_frame"
#define TARGET_NAME  "target"

using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "followme_filtering_node");
  ros::NodeHandle nh;

  Eigen::Vector7d interception_pose;
  initPose(interception_pose);

  // Adjust the camera w.r.t world
  //  Eigen::Matrix3d world_R_camera, Rz, Ry;
  Eigen::Isometry3d world_T_camera = Eigen::Isometry3d::Identity();
  //rpyToRot(0,M_PI/2,0,world_R_camera);
  //yawToRot(0,Rz); //(-M_PI/2,Rz)
  //pitchToRot(0,Ry); //(-M_PI/2,Ry)
  //world_R_camera = Ry * Rz;
  //world_T_camera = world_R_camera * world_T_camera;

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
