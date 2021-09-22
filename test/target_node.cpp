#include "target_estimation/target_manager_ros.hpp"

#define WORLD_FRAME "world"
#define CAMERA_FRAME "camera_depth_optical_frame"

using namespace std;
using namespace rt_logger;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_node");
  ros::NodeHandle nh;

  //target_estimation::TargetEstimation target_estimation_msg;
  //target_estimation_msg.header.frame_id = "world";

  Eigen::Vector3d interception_sphere_pos; // w.r.t world
  interception_sphere_pos << 0.0, 0.0, 0.3;
  double interception_sphere_radius = 1.0;

  RosTargetManager ros_manager(nh);
  ros_manager.setInterceptionSphere(interception_sphere_pos,interception_sphere_radius);
  ros_manager.setTargetTokenName("keyboard"); // FIXME hardcoded
  ros_manager.setReferenceFrame("camera_depth_optical_frame"); // FIXME hardcoded

  //visualization_msgs::Marker sphere_marker;
  //sphere_marker.header.frame_id = manager.getWorldNameFrame();
  //sphere_marker.id = 0;
  //sphere_marker.type = visualization_msgs::Marker::SPHERE;
  //sphere_marker.pose.position.x = interception_sphere_pos(0);
  //sphere_marker.pose.position.y = interception_sphere_pos(1);
  //sphere_marker.pose.position.z = interception_sphere_pos(2);
  //sphere_marker.scale.x  = sphere_marker.scale.y = sphere_marker.scale.z = 2*interception_sphere_radius;
  //sphere_marker.color.r = 0.0f;
  //sphere_marker.color.g = 1.0f;
  //sphere_marker.color.b = 0.0f;
  //sphere_marker.color.a = 0.2;
  //
  //visualization_msgs::Marker target_marker;
  //target_marker.header.frame_id = manager.getWorldNameFrame();
  //target_marker.id = 1;
  //target_marker.type = visualization_msgs::Marker::CUBE;
  //target_marker.scale.x  = target_marker.scale.y = target_marker.scale.z = 0.3;
  //target_marker.color.r = 1.0f;
  //target_marker.color.g = 0.0f;
  //target_marker.color.b = 0.0f;
  //target_marker.color.a = 0.5;
  //
  //visualization_msgs::Marker target_sphere_marker;
  //target_sphere_marker.header.frame_id = manager.getWorldNameFrame();
  //target_sphere_marker.id = 2;
  //target_sphere_marker.type = visualization_msgs::Marker::SPHERE;
  //target_sphere_marker.scale.x  = target_sphere_marker.scale.y = target_sphere_marker.scale.z = 0.1;
  //target_sphere_marker.color.r = 0.0f;
  //target_sphere_marker.color.g = 0.0f;
  //target_sphere_marker.color.b = 1.0f;
  //target_sphere_marker.color.a = 1.0;
  //
  //// Create the ros subscribers and publishers
  //ros::Publisher target_estimation_pub    = nh.advertise<target_estimation::TargetEstimation>("target_estimation", 1);
  //ros::Publisher sphere_marker_pub        = nh.advertise<visualization_msgs::Marker>("sphere_marker", 1);
  //ros::Publisher target_marker_pub        = nh.advertise<visualization_msgs::Marker>("target_marker", 1);
  //ros::Publisher target_sphere_marker_pub = nh.advertise<visualization_msgs::Marker>("target_sphere_marker", 1);
  //tf::TransformBroadcaster br;
  //tf::Transform transform;
  //tf::Quaternion q;

  double f = 50; // Hz -> remember to use the corresponding YAML file
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
