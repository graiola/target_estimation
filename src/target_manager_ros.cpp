#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;
using namespace rt_logger;

RosTargetManager::RosTargetManager(ros::NodeHandle& nh)
{
    //FIXME: how to subscribe to tf messages generated from rosbag -> do I need to run the bag file from the code or not?
    nh_ = nh;
    measurament_sub_ = nh_.subscribe("tf/transform_broadcaster",1,&RosTargetManager::MeasurementCallBack, this);
//    measurament_sub_ = nh_.subscribe("/gazebo/model_states", 1, &RosTargetManager::MeasurementCallBack, this); // FIXME: use tf topic from the rosbag
//    nh_ MUST BE A SUBSCRIBER TO /TF TOPICS

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

void RosTargetManager::setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius)
{
    manager_.setInterceptionSphere(pos,radius);
}


void RosTargetManager::update(const double& dt)
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


//void RosTargetManager::MeasurementCallBack(const gazebo_msgs::ModelStates& model_states) // FIXME: create a thread safe updates
//{
//    int n_models = model_states.name.size();
//    int i_model;
//    bool model_found = false;

//    double t = ros::Time::now().toSec();

//    dt_ = t - t_prev_;

//    for(i_model = 0; i_model<n_models; i_model++)
//    {
//        std::string i_model_name(model_states.name[i_model]);
//        if(i_model_name.compare(model_name_)==0)
//        {
//            model_found = true;
//            break;
//        }
//    }

//    if (model_found)
//    {

//        real_pose_(0) = model_states.pose[i_model].position.x;
//        real_pose_(1) = model_states.pose[i_model].position.y;
//        real_pose_(2) = model_states.pose[i_model].position.z;

//        POSE_qx(real_pose_) = model_states.pose[i_model].orientation.x;
//        POSE_qy(real_pose_) = model_states.pose[i_model].orientation.y;
//        POSE_qz(real_pose_) = model_states.pose[i_model].orientation.z;
//        POSE_qw(real_pose_) = model_states.pose[i_model].orientation.w;

//        real_twist_(0) = model_states.twist[i_model].linear.x;
//        real_twist_(1) = model_states.twist[i_model].linear.y;
//        real_twist_(2) = model_states.twist[i_model].linear.z;
//        real_twist_(3) = model_states.twist[i_model].angular.x;
//        real_twist_(4) = model_states.twist[i_model].angular.y;
//        real_twist_(5) = model_states.twist[i_model].angular.z;
//        //meas_lock.lock();
//#ifdef ADD_NOISE
//        meas_pose_.x() = real_pose_.x() + normal_dist_pos(generator);
//        meas_pose_.y() = real_pose_.y() + normal_dist_pos(generator);
//        meas_pose_.z() = real_pose_.z() + normal_dist_pos(generator);
//        // FIXME Missing orientation noise
//        POSE_qx(meas_pose_) = POSE_qx(real_pose_);
//        POSE_qy(meas_pose_) = POSE_qy(real_pose_);
//        POSE_qz(meas_pose_) = POSE_qz(real_pose_);
//        POSE_qw(meas_pose_) = POSE_qw(real_pose_);
//#else
//        meas_pose_.x() = real_pose_.x();
//        meas_pose_.y() = real_pose_.y();
//        meas_pose_.z() = real_pose_.z();
//        POSE_qx(meas_pose_) = POSE_qx(real_pose_);
//        POSE_qy(meas_pose_) = POSE_qy(real_pose_);
//        POSE_qz(meas_pose_) = POSE_qz(real_pose_);
//        POSE_qw(meas_pose_) = POSE_qw(real_pose_);
//#endif
//        new_meas_ = true;
//        //meas_lock.unlock();

//    }
//    else // No meas
//    {
//        ROS_WARN("No observation!");
//        //meas_lock.lock();
//        new_meas_ = false;
//        //meas_lock.unlock();
//    }

//    update(dt_);

//    t_prev_ = t;

//}

void RosTargetManager::MeasurementCallBack() // FIXME: create a thread safe updates
{
//    int n_models = model_states.name.size();
    int n_models = 1;
    int i_model;
    bool model_found = false;

    double t = ros::Time::now().toSec();

    dt_ = t - t_prev_;

    for(i_model = 0; i_model<n_models; i_model++)
    {
//        std::string i_model_name(model_states.name[i_model]); // model_name_
        std::string i_model_name(model_name_);
        if(i_model_name.compare(model_name_)==0)
        {
            model_found = true;
            break;
        }
    }

    if (model_found)
    {

        // READ THIS FROM MEASUREMENS (/TF MESSAGES)
//        real_pose_(0) = model_states.pose[i_model].position.x;
//        real_pose_(1) = model_states.pose[i_model].position.y;
//        real_pose_(2) = model_states.pose[i_model].position.z;

//        POSE_qx(real_pose_) = model_states.pose[i_model].orientation.x;
//        POSE_qy(real_pose_) = model_states.pose[i_model].orientation.y;
//        POSE_qz(real_pose_) = model_states.pose[i_model].orientation.z;
//        POSE_qw(real_pose_) = model_states.pose[i_model].orientation.w;

//        real_twist_(0) = model_states.twist[i_model].linear.x;
//        real_twist_(1) = model_states.twist[i_model].linear.y;
//        real_twist_(2) = model_states.twist[i_model].linear.z;
//        real_twist_(3) = model_states.twist[i_model].angular.x;
//        real_twist_(4) = model_states.twist[i_model].angular.y;
//        real_twist_(5) = model_states.twist[i_model].angular.z;
        //meas_lock.lock();


        meas_pose_.x() = real_pose_.x();
        meas_pose_.y() = real_pose_.y();
        meas_pose_.z() = real_pose_.z();
        POSE_qx(meas_pose_) = POSE_qx(real_pose_);
        POSE_qy(meas_pose_) = POSE_qy(real_pose_);
        POSE_qz(meas_pose_) = POSE_qz(real_pose_);
        POSE_qw(meas_pose_) = POSE_qw(real_pose_);

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


bool RosTargetManager::parseSquareMatrix(const ros::NodeHandle& n, const std::string& matrix, Eigen::MatrixXd& M)
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

bool RosTargetManager::parseTargetType(const ros::NodeHandle& n, TargetManager::target_t& type)
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

