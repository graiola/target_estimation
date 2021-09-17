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

    // Single Target Management
    Eigen::Vector7d getEstimatedPose()           {return est_pose_;}
    Eigen::Vector6d getEstimatedTwist()          {return est_twist_;}
    Eigen::Vector7d getInterceptionPose()        {return interception_pose_;}
    Eigen::Vector3d getEstimatedPosition()       {return est_position_;}
    Eigen::Quaterniond getEstimatedOrientation() {return est_quaternion_;}
    Eigen::Vector3d getEstimatedRPY()            {return est_rpy_;}
    bool isTargetConverged()                     {return target_converged_;}

    // Multiple Targets Management
    std::map<std::string, Eigen::Vector7d> getEstimatedPose_multi()             {return map_estimated_pose_;} // Map containing estimated pose
    std::map<std::string, Eigen::Vector6d> getEstimatedTwist_multi()            {return map_estimated_twist_;} // Map containing estimated twist
    std::map<std::string, Eigen::Quaterniond> getEstimatedOrientation_multi()   {return map_estimated_quaternion_;} // Map containing estimated quaternions
    std::map<std::string, Eigen::Vector3d> getEstimatedRPY_multi()              {return map_estimated_rpy_;} // Map containing estimated RPY
    std::map<std::string, Eigen::Vector3d> getEstimatedPosition_multi()         {return map_estimated_position_;} // Map containing estimated position
    std::map<std::string, Eigen::Vector7d> getInterceptionPose_multi()           {return map_interception_pose_;} // Map containing intereception pose
    std::map<std::string, bool> isTargetConverged_multi()                       {return map_targets_converged_;} // Map containing intereception pose

    void update(const double& dt);
    void update_v2(const double& dt);

    std::string getTargetTokenFrame()   {return target_name_frame_;}
    std::string getWorldNameFrame()     {return world_name_frame_;}

    int getNumberOfTargets()  {return map_measured_pose_.size();}


private:

    void MeasurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg);
    void MeasurementCallBack_v2(const tf2_msgs::TFMessage::ConstPtr& pose_msg);
    void MeasurementCallBack_multi(const tf2_msgs::TFMessage::ConstPtr& pose_msg);

    bool parseSquareMatrix(const ros::NodeHandle& n, const std::string& matrix, Eigen::MatrixXd& M);
    bool parseTargetType(const ros::NodeHandle& n, TargetManager::target_t& type);

    // put this function in the utils class

//    void initPoseWithMeas();

    ros::NodeHandle nh_;
    TargetManager manager_;
    ros::Subscriber measurament_sub_;

    // FIXME: when a new bag is available remember to change target_name_frame_ into "keyboard"
    // NB: remember to add "/" to target name when pusblishing to /tf topic
    std::string target_name_frame_ = "keyboard1";
    std::string world_name_frame_ = "camera_depth_optical_frame";

    // single target managment
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
    unsigned int target_id_, n_, m_;
    bool target_converged_;

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
    TargetManager::target_t type_;
    std::atomic<bool> new_meas_;
    std::mutex meas_lock;

    // multiple targets managment
    std::map<std::string, Eigen::Vector7d> map_measured_pose_; // Map contained measured pose
    std::map<std::string, unsigned int> map_id_targets_; // Map containing target ID
    std::map<std::string, Eigen::Vector7d> map_estimated_pose_; // Map containing estimated pose
    std::map<std::string, Eigen::Vector6d> map_estimated_twist_; // Map containing estimated twist
    std::map<std::string, Eigen::Quaterniond> map_estimated_quaternion_; // Map containing estimated quaternions
    std::map<std::string, Eigen::Vector3d> map_estimated_rpy_; // Map containing estimated RPY
    std::map<std::string, Eigen::Vector3d> map_estimated_position_; // Map containing estimated position
    std::map<std::string, Eigen::Vector7d> map_pose_error_; // Map containing pose error
    std::map<std::string, Eigen::Vector6d> map_twist_error_; // Map containing twist error
    std::map<std::string, Eigen::Vector7d> map_interception_pose_; // Map containing intereception pose
    std::map<std::string, bool> map_targets_converged_; // Map containing target convergence

    double t_call_, t_pre_call_;

    unsigned int n_active_frames_;
    std::vector<std::string> list_active_frames_;

};

#endif
