#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;
using namespace rt_logger;

//#define DEBUG
#define DEBUG_tmp
#define SINGLE_TARGET_INPUT
//#define MULTI_TARGET_INPUT


RosTargetManager::RosTargetManager(ros::NodeHandle& nh)
{
  // subscribe to /tf topic
  nh_ = nh;
  measurament_sub_ = nh_.subscribe("/tf", 1 , &RosTargetManager::MeasurementCallBack_v2, this);


  // FIXME -> do not initialize the pose. The init must be done using the init of the update soon after the measurement
//  model_name_ = "target";
  target_converged_ = false;
  target_id_ = 0;
  new_meas_ = false;


  t_ = 0.0;
  dt_ = 0.01;
  t_prev_ = 0.0;
  pos_th_ = 0.001;
  ang_th_ = 0.001;

  // tmp
  t_pre_call_ = 0;
  t_call_ = 0;

  n_ = static_cast<unsigned int>(Q_.rows());
  m_ = static_cast<unsigned int>(R_.rows());

  real_twist_.setZero();
  est_twist_.setZero();
  initPose(est_pose_);
  initPose(interception_pose_);
  initPose(meas_pose_);
  initPose(real_pose_);
  sigma_.resize(n_);
  // FIXME

  // Load the parameters for initializing the KF
  if(!parseSquareMatrix(nh_,"Q",Q_) || !parseSquareMatrix(nh_,"R",R_) || !parseSquareMatrix(nh_,"P",P_))
    throw std::runtime_error("Can not load the Cov Matrices!");

  if(!parseTargetType(nh_,type_))
    throw std::runtime_error("Can not load filter type!");

  //  std::string node_name = "/target_node";
  std::string node_name = "/multi_target_node";

  // Create the logger publishers
  RtLogger::getLogger().addPublisher(node_name,twist_error_,"err_twist" );
  RtLogger::getLogger().addPublisher(node_name,meas_pose_  ,"meas_pose" );
  RtLogger::getLogger().addPublisher(node_name,real_pose_  ,"pose"      );
  RtLogger::getLogger().addPublisher(node_name,pose_error_ ,"err_pose"  );
  RtLogger::getLogger().addPublisher(node_name,real_twist_ ,"twist"     );
  RtLogger::getLogger().addPublisher(node_name,est_pose_   ,"est_pose"  );
  RtLogger::getLogger().addPublisher(node_name,est_twist_  ,"est_twist" );
  RtLogger::getLogger().addPublisher(node_name,sigma_      ,"sigma"     );
}

RosTargetManager::RosTargetManager(ros::NodeHandle& nh, std::string& target_name_frame, double& dt)
{
  // subscribe to /tf topic
  nh_ = nh;
  measurament_sub_ = nh_.subscribe("/tf", 1 , &RosTargetManager::MeasurementCallBack, this);

  model_name_ = "/" + target_name_frame;
  std::string in_target = "/" + target_name_frame;
  target_id_ = 0;

  target_converged_ = false;
  new_meas_ = false;

  t_ = 0.0;
  dt_ = dt;
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
  sigma_.resize(n_);

  real_twist_.setZero();
  est_twist_.setZero();
  initPose(est_pose_);
  initPose(interception_pose_);
  initPose(meas_pose_);
  initPose(real_pose_);

  //  std::string node_name = "/target_node";
  std::string node_name = "/multi_target_node";

  // Create the logger publishers
  RtLogger::getLogger().addPublisher(node_name,twist_error_,"err_twist" );
  RtLogger::getLogger().addPublisher(node_name,meas_pose_  ,"meas_pose" );
  RtLogger::getLogger().addPublisher(node_name,real_pose_  ,"pose"      );
  RtLogger::getLogger().addPublisher(node_name,pose_error_ ,"err_pose"  );
  RtLogger::getLogger().addPublisher(node_name,real_twist_ ,"twist"     );
  RtLogger::getLogger().addPublisher(node_name,est_pose_   ,"est_pose"  );
  RtLogger::getLogger().addPublisher(node_name,est_twist_  ,"est_twist" );
  RtLogger::getLogger().addPublisher(node_name,sigma_      ,"sigma"     );
}

void RosTargetManager::setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius)
{
  manager_.setInterceptionSphere(pos,radius);
}

void RosTargetManager::MeasurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg)
{
  meas_lock.lock();

  // check if a new target has been identified
  new_meas_ = true;

  // FIXME: get the number of models on the basis of the different target received from /tf topic
  int n_models = 1;

  // TODO: run the KF to all identified targets (n_models -> run a loop)
  int i_model;

  // this is to find the given target
  for(i_model = 0; i_model<n_models; i_model++)
  {
    std::string i_model_name(pose_msg->transforms.data()->child_frame_id);
  }

  Eigen::Vector3d meas_position;
  meas_position(0) = pose_msg->transforms.data()->transform.translation.x;
  meas_position(1) = pose_msg->transforms.data()->transform.translation.y;
  meas_position(2) = pose_msg->transforms.data()->transform.translation.z;

  Eigen::Quaterniond meas_quaternion;
  meas_quaternion.x() = pose_msg->transforms.data()->transform.rotation.x;
  meas_quaternion.y() = pose_msg->transforms.data()->transform.rotation.y;
  meas_quaternion.z() = pose_msg->transforms.data()->transform.rotation.z;
  meas_quaternion.w() = pose_msg->transforms.data()->transform.rotation.w;

  meas_pose_(0) = meas_position.x();
  meas_pose_(1) = meas_position.y();
  meas_pose_(2) = meas_position.z();
  meas_pose_(3) = meas_quaternion.x();
  meas_pose_(4) = meas_quaternion.y();
  meas_pose_(5) = meas_quaternion.z();
  meas_pose_(6) = meas_quaternion.w();

  meas_lock.unlock();
}

/*
 * 1- explore the pose_msg
 * 2- read all messages (for i=1:length(pose_msg) ---> std::string i_target = pose_msg->transforms.data()->child_frame_id; )
 * 3- get te name of all targets
 * 4- assign to the map str-Eigen the value of the read pose
 * 5- assign to the map str-uint the value of the ID
for // esplora il tf tree
  if keyboard
    // Conversione da pose tf a Eigen
    measurements_[target_names[i]] = eigen_pose;

    */
void RosTargetManager::MeasurementCallBack_v2(const tf2_msgs::TFMessage::ConstPtr& pose_msg)
{
  // c'è da distinguere il nodo che pubblica su /tf
  meas_lock.lock();
#ifdef DEBUG
  double t_call = ros::Time::now().toNSec();
  double dt_call = t_call - t_pre_call_;
  t_pre_call_ = t_call;
  std::cout << "Meas Callback v2 - Delta t [ms] = " << dt_call/1e6 << std::endl;
#endif

  // 1- explore the tf message -> To be implemented
  // 2- read all messages -> to be implemented. For now let's assume a single child target
  int n_targets = 1; // change this once a bag with multiple child-frames are available

#ifdef DEBUG
  std::cout << " --- Measurement Callback v2 --- " << std::ent_pre_calldl;
#endif

  for(int i=0; i<n_targets; i++)
  {
    // 3- read the name of each target
    std::string i_target_name = pose_msg->transforms.data()->child_frame_id;
    std::string delimiter = "_";
    std::vector<std::string> tokens = splitString(i_target_name, delimiter);
    std::string token = tokens[0]; // token is target_name_fr
    std::string target_id_num;
    if(tokens.size()>1)
    {
      target_id_num = tokens[1];
    }

#ifdef DEBUG_tmp
    std::cout << " ------ Token read: " << token << std::endl;
    std::cout << " ------ Token theoretical: " << target_name_frame_ << std::endl;

    for(size_t j=0; j< tokens.size(); j++)
    {
          std::cout << " ------ Tokens.at(j) read: " << tokens.at(j) << std::endl;
          std::cout << " ------ Tokens[j] read: " << tokens[j] << std::endl;
          std::cout << " ------ Tokens[0] read: " << tokens[0] << std::endl;
    }
#endif

    if(token==target_name_frame_)
    {
      // 3- read data from each target
      Eigen::Vector7d meas_pose;
      meas_pose(0) = pose_msg->transforms.data()->transform.translation.x;
      meas_pose(1) = pose_msg->transforms.data()->transform.translation.y;
      meas_pose(2) = pose_msg->transforms.data()->transform.translation.z;
      meas_pose(3) = pose_msg->transforms.data()->transform.rotation.x;
      meas_pose(4) = pose_msg->transforms.data()->transform.rotation.y;
      meas_pose(5) = pose_msg->transforms.data()->transform.rotation.z;
      meas_pose(6) = pose_msg->transforms.data()->transform.rotation.w;

#ifdef SINGLE_TARGET_INPUT // from bag file
      const std::string target_name = token + delimiter + to_string(i);
#endif

#ifdef MULTI_TARGET_INPUT  // from bag file
      const std::string target_name = token + delimiter + target_id_num; // change tokens[1] with target_id_num
#endif

      // 4- assign target data to target name within the map
      map_measured_pose_[target_name] = meas_pose;
      // 5- assign target name to target ID within the map
#ifdef SINGLE_TARGET_INPUT
      map_id_targets_[target_name] = static_cast<unsigned int>(i);
#endif

#ifdef MULTI_TARGET_INPUT
      map_id_targets_[target_name] = stoi(target_id_num);
#endif

#ifdef DEBUG
      std::cout << " ------ Key name: " << target_name << std::endl;
#endif
    }

    if(n_targets>0)
    {
      new_meas_ = true;
    }

  }

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

/*
 * 1- try_lock
 * 2- explore all available targets, i.e. the ones contained in maps map_measured_pose_ and map_id_targets_
 * 3- for each target chek if it has been already initialized. If not init and then update using KF, otherwise update the state with KF
 * 4- lock
 * */

void RosTargetManager::update_v2(const double& dt)
{
  if(meas_lock.try_lock())
  {
    double t = ros::Time::now().toSec();
    dt_ = t - t_prev_;

    // TODO: for all targets -> apply the KF (loop on map of tagets length)
#ifdef DEBUG
    std::cout << " ----- Measured Pose Length = " << map_measured_pose_.size() << std::endl;
    std::cout << " ----- Target ID Length = " << map_id_targets_.size() << std::endl;
#endif

    // Update the state if targets are available
    if( (map_measured_pose_.size() == map_id_targets_.size() ) && ( map_measured_pose_.size() != 0 ) )
    {
      auto it_id = map_id_targets_.begin();
      for(auto it_pose = map_measured_pose_.begin(); it_pose != map_measured_pose_.end(); it_pose++)
      {
#ifdef DEBUG
        std::cout << "Map of Targets ---- Key: " << it_pose->first << " - Value: " << it_pose->second << std::endl;
        std::cout << "Map of Target IDs v2 ---- Key: " << it_id->first << " - Value: " << it_id->second << std::endl;
#endif

        unsigned int target_id =it_id->second;
        Eigen::Vector7d meas_pose = it_pose->second;

        if(new_meas_)
        {
          // check target existence
          if( manager_.getTarget(target_id)==nullptr )
          {
            // init the target
            double dt0 = dt_;
            double t0 = 0.0;
            manager_.init(target_id,dt0,Q_,R_,P_,meas_pose,t0,type_);
          }

          // update with measurements
          manager_.update(target_id,dt,meas_pose);
          new_meas_ = false;
        }
        else
        {
          // if the target has not been already initialized and no measurements are available, the update is not called

          // update without measurements
          manager_.update(target_id,dt);
        }

        it_id++; // update the ID map
      }
    }
    else
    {
      std::cerr << "update_v2 (Update) - The map containg the list of target with pose and the one containing the list of target with ID do not have the same length, or no targets are availabe. Check when add itemes" << std::endl;
    }


    // Set Interception pose
    if( (map_measured_pose_.size() == map_id_targets_.size() ) && ( map_measured_pose_.size() != 0 ) )
    {
      auto it_id = map_id_targets_.begin();
      for(auto it_pose = map_measured_pose_.begin(); it_pose != map_measured_pose_.end(); it_pose++)
      {
        unsigned int target_id =it_id->second;
        Eigen::Vector7d meas_pose = it_pose->second;
        std::string target_key = it_pose->first;

        map_estimated_pose_[target_key]       = manager_.getTarget(target_id)->getEstimatedPose();
        map_estimated_twist_[target_key]      = manager_.getTarget(target_id)->getEstimatedTwist();
        map_estimated_quaternion_[target_key] = manager_.getTarget(target_id)->getEstimatedOrientation();
        map_estimated_rpy_[target_key]        = manager_.getTarget(target_id)->getEstimatedRPY();
        map_estimated_position_[target_key]   = manager_.getTarget(target_id)->getEstimatedPosition();

        // Get the interception point
        if( manager_.getInterceptionPose(target_id, t_, pos_th_, ang_th_, map_interception_pose_[target_key]) )
        {
          map_targets_converged_[target_key] = true;
        }
        else
        {
          map_targets_converged_[target_key] = false;
        }



        t_= t_ + dt;
      }
    }
    else
    {
      std::cerr << "update_v2 (Interception) - The map containg the list of target with pose and the one containing the list of target with ID do not have the same length. Check when add itemes" << std::endl;
    }

    // update the time-related variables
    t_prev_ = t;

    meas_lock.unlock();
  }
}

