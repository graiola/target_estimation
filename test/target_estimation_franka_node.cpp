#include "target_estimation/target_manager_ros.hpp"

//#define DEBUG_CLASS
//#define DEBUG
#define DEBUG_tmp
#define TOADD

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "target_estimation_franka_node");
  ros::NodeHandle nh;

  string world_name_frame;
  string target_token_frame;
  string camera_name_frame;

  Eigen::Vector3d interception_sphere_pos; // w.r.t world
  interception_sphere_pos << 0.0, 0.0, 0.0;
  double interception_sphere_radius = 0.3;

  // Create the ros subscribers and publishers
  ros::Publisher sphere_marker_pub = nh.advertise<visualization_msgs::Marker>("sphere_marker", 1);
//  ros::Publisher target_estimation_pub    = nh.advertise<target_estimation::TargetEstimation>("target_estimation", 1); // not used.

  tf::TransformBroadcaster br;

  double f{1000.00}; // Hz -> remember to use the corresponding YAML file
  double dt{1.0/f};

  std::string topic_to_publish = "/tf";
  std::string robot_topic_eq_pose = "robot_equlibrium_pose_topic";

  RosTargetManager manager(nh, dt, br, topic_to_publish, robot_topic_eq_pose);
  manager.setInterceptionSphere(interception_sphere_pos,interception_sphere_radius);

  camera_name_frame = "camera_depth_optical_frame";
  world_name_frame = "world";
  target_token_frame = "target";
//  target_token_frame = "keyboard";

  manager.setWorldFrameName(world_name_frame);
  manager.setTargetFrameToken(target_token_frame);
  manager.setCameraFrame(camera_name_frame);

  visualization_msgs::Marker sphere_marker;
  sphere_marker.header.frame_id = manager.getWorldNameFrame();
  sphere_marker.id = 0;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.pose.position.x = interception_sphere_pos(0);
  sphere_marker.pose.position.y = interception_sphere_pos(1);
  sphere_marker.pose.position.z = interception_sphere_pos(2);
  sphere_marker.scale.x  = sphere_marker.scale.y = sphere_marker.scale.z = 2*interception_sphere_radius;
  sphere_marker.color.r = 0.0f;
  sphere_marker.color.g = 1.0f;
  sphere_marker.color.b = 0.0f;
  sphere_marker.color.a = 0.2f;

  double t, t_pre = 0;
  int n_targets{0};
  n_targets = manager.getNumberOfTargets();

  unsigned int count{0};

  ros::Rate rate(f);
  while( ros::ok() )
  {
    t = ros::Time::now().toSec();

    dt = t - t_pre;

    // Update the model (the upate method is in charge of publishing also to "tf" and the robot equilibrium point
    manager.update(dt, count);
    n_targets = manager.getNumberOfTargets();

#ifdef DEBUG
    std::cout << "Number of targets: " << n_targets << std::endl;
#endif

    sphere_marker_pub.publish(sphere_marker);

    t_pre = t;

    count++;

    ros::spinOnce();
    rate.sleep();
  }
}
