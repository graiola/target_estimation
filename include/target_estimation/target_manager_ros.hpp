#ifndef TARGET_MANAGER_ROS_HPP
#define TARGET_MANAGER_ROS_HPP

#include <iostream>
#include <iomanip>
#include <vector>

// remove this
//#include <gazebo_msgs/ModelStates.h>

#include <target_estimation/TargetEstimation.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <rt_logger/rt_logger.h>
#include <atomic>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include "target_estimation/target_manager.hpp"

class RosTargetManager
{

public:

    RosTargetManager(ros::NodeHandle& nh);
    RosTargetManager(ros::NodeHandle& nh, std::string& target_name_frame, double& dt);

    void setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius);

    Eigen::Vector7d getEstimatedPose()           {return est_pose_;}
    Eigen::Vector6d getEstimatedTwist()          {return est_twist_;}
    Eigen::Vector7d getInterceptionPose()        {return interception_pose_;}
    Eigen::Vector3d getEstimatedPosition()       {return est_position_;}
    Eigen::Quaterniond getEstimatedOrientation() {return est_quaternion_;}
    Eigen::Vector3d getEstimatedRPY()            {return est_rpy_;}
    bool isTargetConverged()                     {return target_converged_;}


    void update(const double& dt);


private:

    void MeasurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg);
    bool parseSquareMatrix(const ros::NodeHandle& n, const std::string& matrix, Eigen::MatrixXd& M);
    bool parseTargetType(const ros::NodeHandle& n, TargetManager::target_t& type);

//    void initPoseWithMeas();

    ros::NodeHandle nh_;
    TargetManager manager_;
    ros::Subscriber measurament_sub_;
    Eigen::Vector7d real_pose_;
    Eigen::Vector7d meas_pose_;
    Eigen::Quaterniond est_quaternion_;
    Eigen::Vector3d est_position_;
    Eigen::Vector6d real_twist_;
    Eigen::Vector7d est_pose_;
    Eigen::Vector6d est_twist_;
    Eigen::Vector3d est_rpy_;
    Eigen::Vector7d pose_error_;
    Eigen::Vector6d twist_error_;
    Eigen::Vector7d interception_pose_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd R_;
    Eigen::VectorXd sigma_;
    std::string model_name_;
    double pos_th_;
    double ang_th_;
    double t_;
    double t_prev_;
    double dt_;
    unsigned int target_id_, n_, m_;
    bool target_converged_;
    TargetManager::target_t type_;
    std::atomic<bool> new_meas_;
    std::mutex meas_lock;

    std::map<std::string, unsigned int> map_targets_;
//    std::vector<std::map<std::string, unsigned int>> map_targets;
    std::vector<std::string> targets_frames_;
};

#endif
