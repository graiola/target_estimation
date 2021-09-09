#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;
using namespace rt_logger;

RosTargetManager::RosTargetManager(ros::NodeHandle& nh)
{
  // subscribe to /tf topic
  nh_ = nh;
  measurament_sub_ = nh_.subscribe("/tf", 1 , &RosTargetManager::MeasurementCallBack, this);


  model_name_ = "target";
  target_converged_ = false;
  target_id_ = 0;
  new_meas_ = false;
  t_ = 0.0;
  dt_ = 0.01;
  t_prev_ = 0.0;
  pos_th_ = 0.001;
  ang_th_ = 0.001;


  // Load the parameters for initializing the KF
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

  double t = ros::Time::now().toSec();
  dt_ = t - t_prev_;


  if(new_meas_)
  {
    if(manager_.getTarget(target_id_)==nullptr)
    {
      double dt0 = dt_;
      double t0 = 0.0;
      manager_.init(target_id_,dt0,Q_,R_,P_,meas_pose_,t0,type_);
    }

    manager_.update(target_id_,dt,meas_pose_);
    new_meas_ = false;
  }
  else
  {
    manager_.update(target_id_,dt);
  }


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

    pose_error_ = computePoseError(est_pose_,real_pose_);
    twist_error_ << est_twist_ - real_twist_;

    auto kf = manager_.getTarget(target_id_)->getEstimator();

    for (unsigned int i=0; i<n_; i++)
    {
      sigma_(i) = kf->getP()(i,i);
    }

    RtLogger::getLogger().publish(ros::Time::now());

    t_prev_ = t;
  }
}

void RosTargetManager::MeasurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg)
{
  // associo il nome che vedo all'id ->std::map
  // se nella map non esiste il nome -> creo l'oggetto
  // if(target)
  //      manage_.init



  meas_lock.lock();

  //  int n_models = pose_msg->transforms.size();

  int i_model;
  bool model_found = false;
  new_meas_ = true;

  // this is to find the given target
  /*
  for(i_model = 0; i_model<n_models; i_model++)
  {
    std::string i_model_name(pose_msg->transforms.data()->header.frame_id);
    if(i_model_name.compare(model_name_)==0)
    {
      model_found = true;
      break;
    }
  }
  */
  // TODO: loop on all targets found from the measurements -> is it ok or do we need to complete a target first to then move to the nex one?


  Eigen::Vector3d meas_position;
  meas_position(0) = pose_msg->transforms.data()->transform.translation.x;
  meas_position(1) = pose_msg->transforms.data()->transform.translation.y;
  meas_position(2) = pose_msg->transforms.data()->transform.translation.z;

  Eigen::Quaterniond meas_quaternion;
  meas_quaternion.x() = pose_msg->transforms.data()->transform.rotation.x;
  meas_quaternion.y() = pose_msg->transforms.data()->transform.rotation.y;
  meas_quaternion.z() = pose_msg->transforms.data()->transform.rotation.z;
  meas_quaternion.w() = pose_msg->transforms.data()->transform.rotation.w;

  // Debug -> assign to the estimation the measurement obtained from the camera
  //  est_position_ = meas_position;
  //  est_quaternion_ = meas_quaternion;

  /*
  meas_pose_.x() =  pose_msg->transforms.data()->transform.translation.x;
  meas_pose_.y() =  pose_msg->transforms.data()->transform.translation.y;
  meas_pose_.z() =  pose_msg->transforms.data()->transform.translation.z;
  POSE_qx(meas_pose_) = pose_msg->transforms.data()->transform.rotation.x;
  POSE_qy(meas_pose_) = pose_msg->transforms.data()->transform.rotation.x;
  POSE_qz(meas_pose_) = pose_msg->transforms.data()->transform.rotation.x;
  POSE_qw(meas_pose_) = pose_msg->transforms.data()->transform.rotation.x;
  */

  meas_pose_(0) =  meas_position.x();
  meas_pose_(1) =  meas_position.y();
  meas_pose_(2) =  meas_position.z();
  meas_pose_(3) = meas_quaternion.x();
  meas_pose_(4) = meas_quaternion.y();
  meas_pose_(5) = meas_quaternion.z();
  meas_pose_(6) = meas_quaternion.w();

  // Call the update function
  update(dt_);

  meas_lock.unlock();
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

