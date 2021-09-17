#include "target_estimation/target_manager_ros.hpp"

//#define DEBUG_CLASS
//#define DEBUG
#define DEBUG_tmp
#define TOADD

using namespace std;
using namespace rt_logger;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_target_node");
  ros::NodeHandle nh;

  target_estimation::TargetEstimation target_estimation_msg;
  target_estimation_msg.header.frame_id = "world";

  string world_name_frame;
  string target_token_frame;

  Eigen::Vector3d interception_sphere_pos; // w.r.t world
  interception_sphere_pos << 0.0, 0.0, 0.3;
  double interception_sphere_radius = 1.0;
  bool target_converged;

  //  RosTargetManager manager(nh, target_name_frame, dt);
  RosTargetManager manager(nh);
  manager.setInterceptionSphere(interception_sphere_pos,interception_sphere_radius);
  world_name_frame = manager.getWorldNameFrame();
  target_token_frame = manager.getTargetTokenFrame();

  // Orient the gripper camera down
  target_estimation_msg.interception_ready.data = false;
  target_estimation_msg.interception_pose.orientation.x = -0.4996018;
  target_estimation_msg.interception_pose.orientation.y = 0.4999998;
  target_estimation_msg.interception_pose.orientation.z = 0.4999998;
  target_estimation_msg.interception_pose.orientation.w = 0.5003982;

  // signle target management
  Eigen::Vector3d target_position;
  Eigen::Quaterniond target_orientation;
  Eigen::Vector6d target_velocity;
  Eigen::Vector7d interception_pose;
  Eigen::Vector3d target_rpy;

  // multiple targets management
  std::map<std::string, Eigen::Vector7d> multi_target_pose; // Map containing estimated pose
  std::map<std::string, Eigen::Vector6d> multi_target_velocity; // Map containing estimated twist
  std::map<std::string, Eigen::Quaterniond> multi_target_orientation; // Map containing estimated quaternions
  std::map<std::string, Eigen::Vector3d> multi_target_rpy; // Map containing estimated RPY
  std::map<std::string, Eigen::Vector3d> multi_target_position; // Map containing estimated position
  std::map<std::string, bool> multi_target_converged; // Map containing if target is converged
  std::map<std::string, Eigen::Vector7d> multi_interception_pose; // Map containing intereception pose

  visualization_msgs::Marker sphere_marker;
  sphere_marker.header.frame_id = "world";
  sphere_marker.id = 0;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.pose.position.x = interception_sphere_pos(0);
  sphere_marker.pose.position.y = interception_sphere_pos(1);
  sphere_marker.pose.position.z = interception_sphere_pos(2);
  sphere_marker.scale.x  = sphere_marker.scale.y = sphere_marker.scale.z = 2*interception_sphere_radius;
  sphere_marker.color.r = 0.0f;
  sphere_marker.color.g = 1.0f;
  sphere_marker.color.b = 0.0f;
  sphere_marker.color.a = 0.2;

  visualization_msgs::Marker target_marker;
  target_marker.header.frame_id = "world";
  target_marker.id = 1;
  target_marker.type = visualization_msgs::Marker::CUBE;
  target_marker.scale.x  = target_marker.scale.y = target_marker.scale.z = 0.3;
  target_marker.color.r = 1.0f;
  target_marker.color.g = 0.0f;
  target_marker.color.b = 0.0f;
  target_marker.color.a = 0.5;

  visualization_msgs::Marker target_sphere_marker;
  target_sphere_marker.header.frame_id = "world";
  target_sphere_marker.id = 2;
  target_sphere_marker.type = visualization_msgs::Marker::SPHERE;
  target_sphere_marker.scale.x  = target_sphere_marker.scale.y = target_sphere_marker.scale.z = 0.1;
  target_sphere_marker.color.r = 0.0f;
  target_sphere_marker.color.g = 0.0f;
  target_sphere_marker.color.b = 1.0f;
  target_sphere_marker.color.a = 1.0;

  // Create the ros subscribers and publishers
  ros::Publisher target_estimation_pub    = nh.advertise<target_estimation::TargetEstimation>("target_estimation", 1);
  ros::Publisher sphere_marker_pub        = nh.advertise<visualization_msgs::Marker>("sphere_marker", 1);
  ros::Publisher target_marker_pub        = nh.advertise<visualization_msgs::Marker>("target_marker", 1);
  ros::Publisher target_sphere_marker_pub = nh.advertise<visualization_msgs::Marker>("target_sphere_marker", 1);
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  double f = 1000; // Hz -> remember to use the corresponding YAML file
  double dt = 1.0/f;

  ros::Rate rate(f);

  double t, t_pre = 0;
  int n_targets = 0;
  n_targets = manager.getNumberOfTargets();

  while (ros::ok())
  {
#ifndef DEBUG_CLASS

    t = ros::Time::now().toSec();

    dt = t - t_pre;

    // Update the model
    manager.update_v2(dt);

    // FIXME: loop on map length
    multi_target_position     = manager.getEstimatedPosition_multi();
    multi_target_velocity     = manager.getEstimatedTwist_multi();
    multi_target_orientation  = manager.getEstimatedOrientation_multi();
    multi_target_rpy          = manager.getEstimatedRPY_multi();
    multi_target_converged    = manager.isTargetConverged_multi();
    multi_interception_pose   = manager.getInterceptionPose_multi();


    auto it_position = multi_target_position.begin();
    auto it_orientation = multi_target_orientation.begin();
#ifdef DEBUG
    cout << "------------ Target Size (position map): " << multi_target_position.size() << endl;
    cout << "------------ Target Size (orientation map): " << multi_target_orientation.size() << endl;
#endif

    // loop through the maps
    if(multi_target_position.size() == multi_target_orientation.size())
    {
      while( ( it_position != multi_target_position.end() ) && ( it_orientation != multi_target_orientation.end() ) )
      {
        // read data for each target
        target_position = multi_target_position[it_position->first];
        target_orientation = multi_target_orientation[it_position->first];
        target_rpy = multi_target_rpy[it_position->first];
        target_velocity = multi_target_velocity[it_position->first];
        target_converged = multi_target_converged[it_position->first];


        // Create the tf transform between /world and /target
        transform.setOrigin(tf::Vector3(target_position.x(),target_position.y(),target_position.z()));
        q.setRPY(target_rpy(0),target_rpy(1),target_rpy(2));
        q.normalize();
        transform.setRotation(q);

//        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), ("/" + world_name_frame), ("/" + target_token_frame + "_est") ));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), (world_name_frame), (target_token_frame + "_est") ));

        // Publish on /target_marker topic
        target_marker.pose.position.x = target_estimation_msg.pose.position.x = target_position.x();
        target_marker.pose.position.y = target_estimation_msg.pose.position.y = target_position.y();
        target_marker.pose.position.z = target_estimation_msg.pose.position.z = target_position.z();
        target_marker.pose.orientation.x = target_estimation_msg.pose.orientation.x = target_orientation.x();
        target_marker.pose.orientation.y = target_estimation_msg.pose.orientation.y =target_orientation.y();
        target_marker.pose.orientation.z = target_estimation_msg.pose.orientation.z =target_orientation.z();
        target_marker.pose.orientation.w = target_estimation_msg.pose.orientation.w =target_orientation.w();

        target_estimation_msg.twist.linear.x = target_velocity(0);
        target_estimation_msg.twist.linear.y = target_velocity(1);
        target_estimation_msg.twist.linear.z = target_velocity(2);

        target_estimation_msg.header.frame_id = target_marker.header.frame_id = it_position->first;
        target_estimation_msg.header.stamp = ros::Time::now();

        if(target_converged)
        {
          interception_pose = multi_interception_pose[it_position->first];

          target_estimation_msg.interception_ready.data = true;
          target_estimation_msg.interception_pose.position.x = target_sphere_marker.pose.position.x = interception_pose.x();
          target_estimation_msg.interception_pose.position.y = target_sphere_marker.pose.position.y = interception_pose.y();
          target_estimation_msg.interception_pose.position.z = target_sphere_marker.pose.position.z = interception_pose.z();

          target_sphere_marker_pub.publish(target_sphere_marker);
        }
        else
        {
          target_estimation_msg.interception_ready.data = false;
        }

#ifdef DEBUG
        if(target_estimation_msg.interception_ready.data)
        {
          cout << "KF applied to target [ " << it_position->first << " ] has converged! Ready to catch it..." << endl;
        }
        else
        {
          cout << "KF applied to target [ " << it_position->first << " ] has not converged!" << endl;
        }
#endif


        sphere_marker_pub.publish(sphere_marker);

        target_marker_pub.publish(target_marker);
        target_estimation_pub.publish(target_estimation_msg);

#ifdef DEBUG
        cout << "------- Target name: " << it_position->first << endl;
        cout << "------- Position (from map)" << it_position->second << endl;
        cout << "------- Position (from Eigen::Vector3d)" << target_position << endl;
#endif

#ifdef DEBUG
        string target_name_p, target_name_o;
        target_name_p = it_position->first;
        target_name_o = it_orientation->first;
        cout << "------- Target name (position map): " << it_position->first << endl;
        cout << "------- Target name (orientation map): " << it_orientation->first << endl;
        cout << "------- compare (=0 -> equal names): " << target_name_p.compare(target_name_o) << endl;
#endif

        it_position ++;
        it_orientation ++;
      }
    }
    else
    {
      cerr << "multi_traget_node (main->while_loop): dimension mismatch between position and orientation maps. Please check when add new targets." << endl;
    }

    t_pre = t;

#endif
    ros::spinOnce();

    rate.sleep();
  }

  return 0;
}
