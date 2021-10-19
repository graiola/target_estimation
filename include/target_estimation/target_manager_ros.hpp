#ifndef TARGET_MANAGER_ROS_HPP
#define TARGET_MANAGER_ROS_HPP

#include <iostream>
#include <iomanip>
#include <vector>

#include <target_estimation/TargetEstimation.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <atomic>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include "geometry_msgs/PoseStamped.h"

#include "target_estimation/target_manager.hpp"
#include "target_estimation/utils.hpp"

class RosTargetManager
{

public:
    RosTargetManager(ros::NodeHandle& nh, double& dt, std::string& topic_to_publish, tf::TransformBroadcaster& br);
    RosTargetManager(ros::NodeHandle& nh, double& dt, tf::TransformBroadcaster& br, std::string& topic_to_publish, std::string& robot_topic_to_publish);

    void setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius);

    targets_map_t getActiveTargets() {return map_targets_;}

    void update(const double& dt, const unsigned int &count);

    std::string getTargetTokenFrame()   {return target_name_frame_;}
    std::string getWorldNameFrame()     {return world_frame_;}

    int getNumberOfTargets()  {return map_targets_.size();}

    void setWorldFrameName(std::string& name);
    void setTargetFrameToken(std::string& token);
    void setRobotTopicEqPose(std::string& topic_name);
    void setCameraFrame(std::string& camera_frame);



private:

    void init(ros::NodeHandle& nh, double& dt, tf::TransformBroadcaster& br, std::string& topic_to_publish, const bool &publish_robot);

    void MeasurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg);

    bool parseSquareMatrix(const ros::NodeHandle& n, const std::string& matrix, Eigen::MatrixXd& M);
    bool parseTargetType(const ros::NodeHandle& n, TargetManager::target_t& type);

    void sendTF(Eigen::Vector7d &pose, std::string &target_name, std::string &world_name, tf::Transform &transform, tf::Quaternion &q, tf::TransformBroadcaster &br);


    void poseToStampedPose(Eigen::Vector3d &position, Eigen::Quaterniond &orientation,
                            geometry_msgs::PoseStamped &eq_pose_pose_stamped_msg, std::string child_frame_name, const unsigned int &count);
    void poseToStampedPose(Eigen::Vector7d &pose, geometry_msgs::PoseStamped &eq_pose_pose_stamped_msg,
                                             std::string child_frame_name, const unsigned int &count);


    ros::NodeHandle nh_;
    TargetManager manager_;
    ros::Subscriber measurament_sub_;
    ros::Publisher robot_eq_pose_pub_;
    ros::Subscriber robot_cart_space_sub_;

    std::string target_name_frame_ = "";
    std::string world_frame_ = "";
    std::string camera_frame_ = "";
    const std::string frame_name_delimiter_ = "_";

    Eigen::MatrixXd Q_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd R_;
    unsigned int n_, m_;
    Eigen::VectorXd sigma_;
    std::string model_name_;
    double pos_th_;
    double ang_th_;
    double t_;
    double t_prev_;
    double dt_;
    TargetManager::target_t type_;
    std::mutex meas_lock;

    // Using a map with structs it does not work well
    targets_map_t map_targets_;

    unsigned int n_active_frames_;

    std::string tf_topic_name_ = "";
    tf::TransformBroadcaster br_;
    tf::Transform transform_;
    tf::Quaternion q_;

    geometry_msgs::PoseStamped robot_eq_pose_msg_;
    std::string robot_topic_pub_ = "";
    bool interface_with_robot_;

};

#endif
