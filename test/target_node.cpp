/**
*
*/


#include <random>

#include "target_estimation/target_manager_ros.hpp"

#define ADD_NOISE
// Simulated measurement noise
const double pObserved_stddev_pos = 0.01; // [m] 0.001[m] = 1[mm]
const double pObserved_mean_pos = 0.0;     // [m]
const double pObserved_stddev_ori = 0.01; // [rad]
const double pObserved_mean_ori = 0.0;    // [rad]
// Statistical generators
std::default_random_engine generator;
std::normal_distribution<double> normal_dist_pos(pObserved_mean_pos, pObserved_stddev_pos);
std::normal_distribution<double> normal_dist_ori(pObserved_mean_ori, pObserved_stddev_ori);

using namespace std;
using namespace rt_logger;

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "target_node");

    ros::NodeHandle nh;

    target_estimation::TargetEstimation target_estimation_msg;
    target_estimation_msg.header.frame_id = "world";

    // Orient the gripper camera down
    target_estimation_msg.interception_ready.data = false;
    target_estimation_msg.interception_pose.orientation.x = -0.4996018;
    target_estimation_msg.interception_pose.orientation.y = 0.4999998;
    target_estimation_msg.interception_pose.orientation.z = 0.4999998;
    target_estimation_msg.interception_pose.orientation.w = 0.5003982;

    Eigen::Vector3d interception_sphere_pos; // w.r.t world
    Eigen::Vector3d target_position;
    Eigen::Quaterniond target_orientation;
    Eigen::Vector6d target_velocity;
    Eigen::Vector7d interception_pose;
    Eigen::Vector3d target_rpy;
    interception_sphere_pos << 0.0, 0.0, 0.3;
    double interception_sphere_radius = 1.0;

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
    ros::Publisher target_estimation_pub    = nh.advertise<target_estimation::TargetEstimation>("target_estimation", 1000);
    ros::Publisher sphere_marker_pub        = nh.advertise<visualization_msgs::Marker>("sphere_marker", 1);
    ros::Publisher target_marker_pub        = nh.advertise<visualization_msgs::Marker>("target_marker", 1);
    ros::Publisher target_sphere_marker_pub = nh.advertise<visualization_msgs::Marker>("target_sphere_marker", 1);
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    RosTargetManager manager(nh);
    manager.setInterceptionSphere(interception_sphere_pos,interception_sphere_radius);

    double f = 500;
    //double dt = 1.0/f;
    ros::Rate rate(f);

    while (ros::ok())
    {

        //manager.update(dt);
        target_position     = manager.getEstimatedPosition();
        target_velocity     = manager.getEstimatedTwist();
        target_orientation  = manager.getEstimatedOrientation();
        target_rpy          = manager.getEstimatedRPY();

        // Create the tf transform between /world and /target
        transform.setOrigin(tf::Vector3(target_position.x(),target_position.y(),target_position.z()));
        //quatToRpy(target_orientation,target_rpy);
        q.setRPY(target_rpy(0),target_rpy(1),target_rpy(2));
        //q.setX(target_orientation.x());
        //q.setY(target_orientation.y());
        //q.setZ(target_orientation.z());
        //q.setW(target_orientation.w());
        q.normalize();
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world" , "/target" ));

        target_marker.pose.position.x = target_estimation_msg.pose.position.x = target_position.x();
        target_marker.pose.position.y = target_estimation_msg.pose.position.y = target_position.y();
        target_marker.pose.position.z = target_estimation_msg.pose.position.z = target_position.z();
        target_marker.pose.orientation.x = target_orientation.x();
        target_marker.pose.orientation.y = target_orientation.y();
        target_marker.pose.orientation.z = target_orientation.z();
        target_marker.pose.orientation.w = target_orientation.w();

        target_estimation_msg.twist.linear.x = target_velocity(0);
        target_estimation_msg.twist.linear.y = target_velocity(1);
        target_estimation_msg.twist.linear.z = target_velocity(2);

        target_marker_pub.publish(target_marker);

        if(manager.isTargetConverged())
        {
            interception_pose = manager.getInterceptionPose();

            target_estimation_msg.interception_ready.data = true;
            target_estimation_msg.interception_pose.position.x = target_sphere_marker.pose.position.x = interception_pose.x();
            target_estimation_msg.interception_pose.position.y = target_sphere_marker.pose.position.y = interception_pose.y();
            target_estimation_msg.interception_pose.position.z = target_sphere_marker.pose.position.z = interception_pose.z();

            target_sphere_marker_pub.publish(target_sphere_marker);
        }
        else
            target_estimation_msg.interception_ready.data = false;


        sphere_marker_pub.publish(sphere_marker);

        target_estimation_pub.publish(target_estimation_msg);

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
