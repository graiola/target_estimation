#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;
using namespace rt_logger;

//#define DEBUG
#define DEBUG_tmp


RosTargetManager::RosTargetManager(ros::NodeHandle& nh)
{
  // subscribe to /tf topic
  nh_ = nh;
  measurament_sub_ = nh_.subscribe("/tf", 1 , &RosTargetManager::MeasurementCallBack, this);

  // FIXME -> do not initialize the pose. The init must be done using the init of the update soon after the measurement
  target_converged_ = false;
  target_id_ = 0;
  n_active_frames_ = 0;

  t_ = 0.0;
  dt_ = 0.001;
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

  // Load the parameters for initializing the KF
  if(!parseSquareMatrix(nh_,"Q",Q_) || !parseSquareMatrix(nh_,"R",R_) || !parseSquareMatrix(nh_,"P",P_))
    throw std::runtime_error("Can not load the Cov Matrices!");

  if(!parseTargetType(nh_,type_))
    throw std::runtime_error("Can not load filter type!");

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

RosTargetManager::RosTargetManager(ros::NodeHandle& nh, double& dt)
{
  // subscribe to /tf topic
  nh_ = nh;
  measurament_sub_ = nh_.subscribe("/tf", 1 , &RosTargetManager::MeasurementCallBack, this);


  // FIXME -> do not initialize the pose. The init must be done using the init of the update soon after the measurement
  target_converged_ = false;
  target_id_ = 0;
  n_active_frames_ = 0;


  t_ = 0.0;
  dt_ = dt;
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

  // Load the parameters for initializing the KF
  if(!parseSquareMatrix(nh_,"Q",Q_) || !parseSquareMatrix(nh_,"R",R_) || !parseSquareMatrix(nh_,"P",P_))
    throw std::runtime_error("Can not load the Cov Matrices!");

  if(!parseTargetType(nh_,type_))
    throw std::runtime_error("Can not load filter type!");

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

  // 1- explore the tf message -> To be implemented
  std::string current_frame = pose_msg->transforms.data()->child_frame_id;

  std::vector<std::string> tokens = splitString(current_frame, frame_name_delimiter_);

  if( (tokens.front() == target_name_frame_) && (tokens.back() != "est") )
  {
    if(map_id_targets_.count(current_frame) != 1)
    {
      // target not found
      map_id_targets_[current_frame] = n_active_frames_;
      n_active_frames_ ++;
    }

    // 3- read data from each target
    Eigen::Vector7d meas_pose;
    meas_pose(0) = pose_msg->transforms.data()->transform.translation.x;
    meas_pose(1) = pose_msg->transforms.data()->transform.translation.y;
    meas_pose(2) = pose_msg->transforms.data()->transform.translation.z;
    meas_pose(3) = pose_msg->transforms.data()->transform.rotation.x;
    meas_pose(4) = pose_msg->transforms.data()->transform.rotation.y;
    meas_pose(5) = pose_msg->transforms.data()->transform.rotation.z;
    meas_pose(6) = pose_msg->transforms.data()->transform.rotation.w;

    // 4- assign target data to target name within the map
    map_measured_pose_[current_frame] = meas_pose;
    map_targets_new_meas_[current_frame] = true;

 #ifdef DEBUG
    std::cout << "Transforms.size = " << pose_msg->transforms.size() << "Map_ID.size = : " << map_id_targets_.size() << "Map_MEAS.size = : " << map_measured_pose_.size() << std::endl;
 #endif
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

/*
 * 1- try_lock
 * 2- explore all available targets, i.e. the ones contained in maps map_measured_pose_ and map_id_targets_
 * 3- for each target chek if it has been already initialized. If not init and then update using KF, otherwise update the state with KF
 * 4- lock
 * */
void RosTargetManager::update(const double& dt)
{
  if(meas_lock.try_lock())
  {
    double t = ros::Time::now().toSec();
    dt_ = t - t_prev_;

    unsigned int target_id;
    Eigen::Vector7d meas_pose;

#ifdef DEBUG
    std::cout << "n of active targets = " << map_measured_pose_.size() << std::endl;
#endif

    // Update the state if targets are available
    if( (map_measured_pose_.size() == map_id_targets_.size() ) && ( map_id_targets_.size() != 0 ) )
    {
      for(auto& it_pose : map_measured_pose_)
      {
        unsigned int target_id = map_id_targets_[it_pose.first];
        Eigen::Vector7d meas_pose = map_measured_pose_[it_pose.first];

        if(map_targets_new_meas_[it_pose.first])
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
          map_targets_new_meas_[it_pose.first] = false;
        }
        else
        {
          // if the target has not been already initialized and no measurements are available, the update is not called

          // update without measurements
          manager_.update(target_id,dt);
        }
      }
    }
    else
    {
      std::cerr << "update_v2 (Update) - The map containg the list of target with pose and the one containing the list of target with ID do not have the same length, or no targets are availabe. Check when add itemes" << std::endl;
    }


    // Set Interception pose
    if( (map_measured_pose_.size() == map_id_targets_.size() ) && ( map_measured_pose_.size() != 0 ) )
    {
      for(auto it_pose : map_measured_pose_)
      {
        target_id = map_id_targets_[it_pose.first];
        meas_pose = map_measured_pose_[it_pose.first];

        map_estimated_pose_[it_pose.first]       = manager_.getTarget(target_id)->getEstimatedPose();
        map_estimated_twist_[it_pose.first]      = manager_.getTarget(target_id)->getEstimatedTwist();
        map_estimated_quaternion_[it_pose.first] = manager_.getTarget(target_id)->getEstimatedOrientation();
        map_estimated_rpy_[it_pose.first]        = manager_.getTarget(target_id)->getEstimatedRPY();
        map_estimated_position_[it_pose.first]   = manager_.getTarget(target_id)->getEstimatedPosition();

        // Get the interception point
        if( manager_.getInterceptionPose(target_id, t_, pos_th_, ang_th_, map_interception_pose_[it_pose.first]) )
        {
          map_targets_converged_[it_pose.first] = true;
        }
        else
        {
          map_targets_converged_[it_pose.first] = false;
        }
      }
    }
    else
    {
      std::cerr << "update_v2 (Interception) - The map containg the list of target with pose and the one containing the list of target with ID do not have the same length. Check when add itemes" << std::endl;
    }

    // update the time-related variables
    t_= t_ + dt;
    t_prev_ = t;

    meas_lock.unlock();
  }
}

void RosTargetManager::updateTargets(std::vector<std::string>& list_active_frames, unsigned int& n_active_frames, std::string& current_frame)
{
  // find cuurent frame in the list of active targets
  std::vector<std::string>::iterator it_list_names = std::find(list_active_frames.begin(), list_active_frames.end(), current_frame);

  if (it_list_names == list_active_frames.end())
  {
#ifdef DEBUG
    std::cout << "New target found to be added: " << current_frame << std::endl;
#endif
    list_active_frames.push_back(current_frame);
    n_active_frames++;
  }
#ifdef DEBUG
  std::cout << "Number of active frames: " << n_active_frames << std::endl;
#endif
}

void RosTargetManager::updateTargetsToken(std::vector<std::string>& list_active_frames, unsigned int& n_active_frames, std::string& current_frame, const std::string& token)
{
  std::vector<std::string> tokens = splitString(current_frame, frame_name_delimiter_);
  if( ( tokens.front() == token)  && tokens.back() != "est")
  {
    updateTargets(list_active_frames, n_active_frames, current_frame);
  }
#ifdef DEBUG_tmp
  std::cout << "Number of active frames: " << n_active_frames << std::endl;
#endif
}

void RosTargetManager::setWorldFrameName(std::string& name)
{
  world_name_frame_ = name;
}

void RosTargetManager::setTargetFrameToken(std::string& token)
{
  target_name_frame_ = token;
}
