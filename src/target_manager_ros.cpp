#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;

//#define DEBUG
#define DEBUG_tmp

RosTargetManager::RosTargetManager(ros::NodeHandle& nh, double& dt, const bool &publish_robot)
{
  // subscribe to /tf topic
  nh_ = nh;
  measurament_sub_ = nh_.subscribe("/tf", 1 , &RosTargetManager::MeasurementCallBack, this);

  if(publish_robot)
  {
    // Publish to Fraka Equilibrium Pose topic
    std::string topic_to_publish = "cartesian_impedance_example_controller/equilibrium_pose";
    franka_eq_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(topic_to_publish, 1, true);
  }

  n_active_frames_ = 0;

  t_ = 0.0;
  dt_ = dt;
  t_prev_ = 0.0;
  pos_th_ = 1;
  ang_th_ = 1;

  // tmp
  t_pre_call_ = 0;
  t_call_ = 0;

  n_ = static_cast<unsigned int>(Q_.rows());
  m_ = static_cast<unsigned int>(R_.rows());

  sigma_.resize(n_);

  // Load the parameters for initializing the KF
  if(!parseSquareMatrix(nh_,"Q",Q_) || !parseSquareMatrix(nh_,"R",R_) || !parseSquareMatrix(nh_,"P",P_))
    throw std::runtime_error("Can not load the Cov Matrices!");

  if(!parseTargetType(nh_,type_))
    throw std::runtime_error("Can not load filter type!");

  std::string node_name = "/multi_target_node";
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
void RosTargetManager::update(const double& dt, const unsigned int &count)
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

        Eigen::Vector7d inteception_pose;

        // Get the interception point
        if( manager_.getInterceptionPose(target_id, t_, pos_th_, ang_th_, inteception_pose) )
        {
          map_targets_converged_[it_pose.first] = true;
          map_interception_pose_[it_pose.first] = inteception_pose;
        }
        else
        {
          map_targets_converged_[it_pose.first] = false;
        }

#ifdef DEBUG_tmp
        std::cout << "Interception pose" << std::endl;
        std::cout << inteception_pose << std::endl;
#endif

        Eigen::Vector3d target_position;
        Eigen::Quaterniond target_orientation;
        target_position = map_estimated_position_[it_pose.first];
        target_orientation = map_estimated_quaternion_[it_pose.first];
        std::string target_name = it_pose.first;

        sendTF(target_position, target_orientation, target_name, world_name_frame_, transform_, q_, br_);

        // -- FIXME --- //
        // send interception pose when properly working
        franka_eq_pose_msg_.header.frame_id = it_pose.first;
        franka_eq_pose_msg_.header.seq =count;
        franka_eq_pose_msg_.pose.position.x=target_position.x();
        franka_eq_pose_msg_.pose.position.y=target_position.y();
        franka_eq_pose_msg_.pose.position.z=target_position.z();
        franka_eq_pose_msg_.pose.orientation.x=target_orientation.x();
        franka_eq_pose_msg_.pose.orientation.y=target_orientation.y();
        franka_eq_pose_msg_.pose.orientation.z=target_orientation.z();
        franka_eq_pose_msg_.pose.orientation.w=target_orientation.w();

        franka_eq_pose_pub_.publish(franka_eq_pose_msg_);
        // --- FIXME --- //

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
#ifdef DEBUG
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

void RosTargetManager::sendTF(Eigen::Vector3d &postion, Eigen::Quaterniond &orientation, std::string &target_name, std::string &world_name, tf::Transform &transform, tf::Quaternion &q, tf::TransformBroadcaster &br)
{
  // send data
  transform.setOrigin(tf::Vector3(postion.x(),postion.y(),postion.z()));
//        q.setRPY(target_rpy(0),target_rpy(1),target_rpy(2));
  q = tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w());
  q.normalize();
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), (world_name), (target_name + "_est") ));
}
