#include "target_estimation/target_manager_ros.hpp"

#define WORLD_FRAME  "world"
#define CAMERA_FRAME "camera_depth_optical_frame"
#define TARGET_NAME  "target"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_node");
  ros::NodeHandle nh;

  //target_estimation::TargetEstimation target_estimation_msg;
  //target_estimation_msg.header.frame_id = "world";

  Eigen::Vector3d interception_sphere_pos; // w.r.t world
  interception_sphere_pos << 0.0, 0.0, 0.0;
  double interception_sphere_radius = 1.0;
  Eigen::Vector7d interception_pose;
  initPose(interception_pose);

  // Adjust the camera w.r.t world
  Eigen::Matrix3d world_R_camera, Rz, Ry;
  Eigen::Isometry3d world_T_camera = Eigen::Isometry3d::Identity();
  //rpyToRot(0,M_PI/2,0,world_R_camera);
  //yawToRot(0,Rz); //(-M_PI/2,Rz)
  //pitchToRot(0,Ry); //(-M_PI/2,Ry)
  //world_R_camera = Ry * Rz;
  //world_T_camera = world_R_camera * world_T_camera;

  RosTargetManager ros_manager(nh);
  ros_manager.setInterceptionSphere(interception_sphere_pos,interception_sphere_radius);
  ros_manager.setTargetTokenName(TARGET_NAME);
  ros_manager.setReferenceFrameName(WORLD_FRAME);
  ros_manager.setObserverFrameName(CAMERA_FRAME);
  ros_manager.setObserverTransform(world_T_camera);

  //  visualization_msgs::Marker sphere_marker;
  //  sphere_marker.header.frame_id = WORLD_FRAME;
  //  sphere_marker.id = 0;
  //  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  //  sphere_marker.pose.position.x = interception_sphere_pos(0);
  //  sphere_marker.pose.position.y = interception_sphere_pos(1);
  //  sphere_marker.pose.position.z = interception_sphere_pos(2);
  //  sphere_marker.scale.x  = sphere_marker.scale.y = sphere_marker.scale.z = 2*interception_sphere_radius;
  //  sphere_marker.color.r = 0.0f;
  //  sphere_marker.color.g = 1.0f;
  //  sphere_marker.color.b = 0.0f;
  //  sphere_marker.color.a = 0.2;

//  visualization_msgs::Marker interception_marker;
//  interception_marker.header.frame_id = WORLD_FRAME;
//  interception_marker.id = 1;
//  interception_marker.type = visualization_msgs::Marker::SPHERE;
//  interception_marker.scale.x  = interception_marker.scale.y = interception_marker.scale.z = 0.1;
//  interception_marker.color.r = 0.0f;
//  interception_marker.color.g = 0.0f;
//  interception_marker.color.b = 1.0f;
//  interception_marker.color.a = 1.0;

  //// Create the ros subscribers and publishers
  ros::Publisher interception_pub             = nh.advertise<visualization_msgs::Marker>("interception", 1);
  ros::Publisher sphere_marker_pub            = nh.advertise<visualization_msgs::Marker>("sphere", 1);

  double f = 50; // Hz -> remember to use the corresponding YAML file
  double dt = 1.0/f;

  ros::Rate rate(f);

  while (ros::ok())
  {

    ros_manager.update(dt);
//    interception_pose = ros_manager.getInterceptionPose();

//    interception_marker.pose.position.x = interception_pose(0);
//    interception_marker.pose.position.y = interception_pose(1);
//    interception_marker.pose.position.z = interception_pose(2);

//    sphere_marker_pub.publish(sphere_marker);
//    interception_pub.publish(interception_marker);

    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
