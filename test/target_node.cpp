/**
*
*/

#include <iostream>
#include <iomanip>
#include <vector>
#include <gazebo_msgs/ModelStates.h>
#include <target_estimation/TargetEstimation.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <rt_logger/rt_logger.h>
#include <atomic>
#include <random>
#include <tf/transform_broadcaster.h>

#include "target_estimation/target_manager.hpp"

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


bool parseSquareMatrix(const ros::NodeHandle& n, const std::string& matrix, Eigen::MatrixXd& M)
{
    std::vector<double> Mv;
    if (n.getParam(matrix, Mv))
    {
        unsigned int size = static_cast<unsigned int>(std::sqrt(Mv.size()));
        M = Eigen::Map<Eigen::MatrixXd>(Mv.data(),size,size);
    }
    else
    {
        ROS_ERROR_STREAM("Can not find matrix: "<<matrix);
        return false;
    }

    return true;
}

bool parseTargetType(const ros::NodeHandle& n, TargetManager::target_t& type)
{
    std::string type_str;
    if (n.getParam("type", type_str))
    {
        if (std::strcmp(type_str.c_str(),"rpy")==0)
            type = TargetManager::target_t::RPY;
        else if (std::strcmp(type_str.c_str(),"rpy_ext") == 0)
            type = TargetManager::target_t::RPY_EXT;

        return true;
    }
    else
    {
        ROS_ERROR_STREAM("Can not find type");
        return false;
    }

}

class RosTargetManager
{

public:

    RosTargetManager(ros::NodeHandle& nh) // FIXME: This should be moved inside the library, it should work as a target ros wrapper, exposing the
    // target's input and output
    {
        nh_ = nh;
        measurament_sub_ = nh_.subscribe("/gazebo/model_states", 1, &RosTargetManager::MeasurementCallBack, this); // FIXME: use tf topic from the rosbag
        model_name_ = "target";
        target_converged_ = false;
        target_id_ = 0;
        new_meas_ = false;
        t_ = 0.0;
        dt_ = 0.01;
        t_prev_ = 0.0;
        pos_th_ = 0.001;
        ang_th_ = 0.001;


        // Load the parameters for the measuraments
        if(!parseSquareMatrix(nh_,"Q",Q_) || !parseSquareMatrix(nh_,"R",R_) || !parseSquareMatrix(nh_,"P",P_))
            throw std::runtime_error("Can not load the Cov Matrices!");

        if(!parseTargetType(nh_,type_))
            throw std::runtime_error("Can not load filter type!");

        n_ = static_cast<unsigned int>(Q_.rows());
        m_ = static_cast<unsigned int>(R_.rows());

        real_twist_.setZero();
        est_twist_.setZero();
        initPose(est_pose_);
        initPose(interception_pose_);
        initPose(meas_pose_);
        initPose(real_pose_);
        sigma_.resize(n_);


        // Create the logger publishers
        RtLogger::getLogger().addPublisher("/target_node",twist_error_,"err_twist" );
        RtLogger::getLogger().addPublisher("/target_node",meas_pose_  ,"meas_pose" );
        RtLogger::getLogger().addPublisher("/target_node",real_pose_  ,"pose"      );
        RtLogger::getLogger().addPublisher("/target_node",pose_error_ ,"err_pose"  );
        RtLogger::getLogger().addPublisher("/target_node",real_twist_ ,"twist"     );
        RtLogger::getLogger().addPublisher("/target_node",est_pose_   ,"est_pose"  );
        RtLogger::getLogger().addPublisher("/target_node",est_twist_  ,"est_twist" );
        RtLogger::getLogger().addPublisher("/target_node",sigma_      ,"sigma"     );
    }

    void setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius)
    {
        manager_.setInterceptionSphere(pos,radius);
    }

    Eigen::Vector7d getEstimatedPose()           {return est_pose_;}
    Eigen::Vector6d getEstimatedTwist()          {return est_twist_;}
    Eigen::Vector7d getInterceptionPose()        {return interception_pose_;}
    Eigen::Vector3d getEstimatedPosition()       {return est_position_;}
    Eigen::Quaterniond getEstimatedOrientation() {return est_quaternion_;}
    Eigen::Vector3d getEstimatedRPY()            {return est_rpy_;}
    bool isTargetConverged()                     {return target_converged_;}


    void update(const double& dt)
    {

        if(new_meas_)
        {
            if(manager_.getTarget(target_id_)==nullptr)
            {
                double dt0 = dt_;
                double t0 = 0.0;
                manager_.init(target_id_,dt0,Q_,R_,P_,meas_pose_,t0,type_);
            }
            manager_.update(target_id_,dt,meas_pose_);
        }
        else
            manager_.update(target_id_,dt);


        if(manager_.getTarget(target_id_)!=nullptr)
        {

            est_pose_       = manager_.getTarget(target_id_)->getEstimatedPose();
            est_twist_      = manager_.getTarget(target_id_)->getEstimatedTwist();
            est_quaternion_ = manager_.getTarget(target_id_)->getEstimatedOrientation();
            est_rpy_        = manager_.getTarget(target_id_)->getEstimatedRPY();
            est_position_   = manager_.getTarget(target_id_)->getEstimatedPosition();

            // Get the interception point
            if(manager_.getInterceptionPose(target_id_,t_,pos_th_,ang_th_,interception_pose_))
                target_converged_ = true;

            else
                target_converged_ = false;

            t_= t_ + dt;
        }


        pose_error_ = computePoseError(est_pose_,real_pose_);
        twist_error_ << est_twist_ - real_twist_;

        auto kf = manager_.getTarget(target_id_)->getEstimator();

        for (unsigned int i=0; i<n_; i++)
            sigma_(i) = kf->getP()(i,i);

        //std::cout<< t_ << std::endl;
        //std::cout<< dt << std::endl;
        //getchar();

        RtLogger::getLogger().publish(ros::Time::now());
    }


private:

    void MeasurementCallBack(const gazebo_msgs::ModelStates& model_states) // FIXME: create a thread safe updates
    {
        int n_models = model_states.name.size();
        int i_model;
        bool model_found = false;

        double t = ros::Time::now().toSec();

        dt_ = t - t_prev_;

        for(i_model = 0; i_model<n_models; i_model++)
        {
            std::string i_model_name(model_states.name[i_model]);
            if(i_model_name.compare(model_name_)==0)
            {
                model_found = true;
                break;
            }
        }

        if (model_found)
        {

            real_pose_(0) = model_states.pose[i_model].position.x;
            real_pose_(1) = model_states.pose[i_model].position.y;
            real_pose_(2) = model_states.pose[i_model].position.z;

            POSE_qx(real_pose_) = model_states.pose[i_model].orientation.x;
            POSE_qy(real_pose_) = model_states.pose[i_model].orientation.y;
            POSE_qz(real_pose_) = model_states.pose[i_model].orientation.z;
            POSE_qw(real_pose_) = model_states.pose[i_model].orientation.w;

            real_twist_(0) = model_states.twist[i_model].linear.x;
            real_twist_(1) = model_states.twist[i_model].linear.y;
            real_twist_(2) = model_states.twist[i_model].linear.z;
            real_twist_(3) = model_states.twist[i_model].angular.x;
            real_twist_(4) = model_states.twist[i_model].angular.y;
            real_twist_(5) = model_states.twist[i_model].angular.z;
            //meas_lock.lock();
#ifdef ADD_NOISE
            meas_pose_.x() = real_pose_.x() + normal_dist_pos(generator);
            meas_pose_.y() = real_pose_.y() + normal_dist_pos(generator);
            meas_pose_.z() = real_pose_.z() + normal_dist_pos(generator);
            // FIXME Missing orientation noise
            POSE_qx(meas_pose_) = POSE_qx(real_pose_);
            POSE_qy(meas_pose_) = POSE_qy(real_pose_);
            POSE_qz(meas_pose_) = POSE_qz(real_pose_);
            POSE_qw(meas_pose_) = POSE_qw(real_pose_);
#else
            meas_pose_.x() = real_pose_.x();
            meas_pose_.y() = real_pose_.y();
            meas_pose_.z() = real_pose_.z();
            POSE_qx(meas_pose_) = POSE_qx(real_pose_);
            POSE_qy(meas_pose_) = POSE_qy(real_pose_);
            POSE_qz(meas_pose_) = POSE_qz(real_pose_);
            POSE_qw(meas_pose_) = POSE_qw(real_pose_);
#endif
            new_meas_ = true;
            //meas_lock.unlock();

        }
        else // No meas
        {
            ROS_WARN("No observation!");
            //meas_lock.lock();
            new_meas_ = false;
            //meas_lock.unlock();
        }

        update(dt_);

        t_prev_ = t;

    }

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
};

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
