#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;

//#define DEBUG
#define DEBUG_tmp

RosTargetManager::RosTargetManager(ros::NodeHandle& nh, double& dt)
{
  publish_to_robot_ = false;

  if( !init(nh, dt, publish_to_robot_) )
  {
    std::cerr << "Cannot Initialize RosTargetManager Class" << std::endl;
  }
}

RosTargetManager::RosTargetManager(ros::NodeHandle& nh, double& dt, std::string& robot_topic_to_publish)
{
  publish_to_robot_ = true;
  setRobotTopicEqPose(robot_topic_to_publish);

  if( !init(nh, dt, publish_to_robot_) )
  {
    std::cerr << "Cannot Initialize RosTargetManager Class" << std::endl;
  }
}

bool RosTargetManager::init(ros::NodeHandle& nh, double& dt, const bool &publish_robot)
{
  bool res;

  // subscribe to /tf topic
  nh_ = nh;
  measurament_sub_ = nh_.subscribe("/tf", 1 , &RosTargetManager::MeasurementCallBack, this);

  if(publish_robot)
  {
    // Publish to Fraka Equilibrium Pose topic
    std::string topic_to_publish = "cartesian_impedance_example_controller/equilibrium_pose";
    topic_to_publish = robot_topic_; // FIXME -> add member to get robot topic
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
  {
    throw std::runtime_error("Can not load the Cov Matrices!");
  }

  if(!parseTargetType(nh_,type_))
  {
    throw std::runtime_error("Can not load filter type!");
  }

  res = true;

  return res;
}

void RosTargetManager::setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius)
{
  manager_.setInterceptionSphere(pos,radius);
}

void RosTargetManager::MeasurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg)
{
  meas_lock.lock();

  // 1- explore the tf message
  std::string current_frame = pose_msg->transforms.data()->child_frame_id;

  std::vector<std::string> tokens = splitString(current_frame, frame_name_delimiter_);

  // 2- check if incoming target already exists, eventually add new one and update the map
  if( (tokens.front() == target_name_frame_) && (tokens.back() != "est") )
  {
    if(map_targets_.count(current_frame) != 1)
    {
      // target not found, so add it!
      map_targets_[current_frame].id = n_active_frames_;

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
    map_targets_[current_frame].measured_pose_ = meas_pose;
    map_targets_[current_frame].frame_name_ = current_frame;
    map_targets_[current_frame].new_meas_ = true;

 #ifdef DEBUG
    std::cout << "Transforms.size = " << pose_msg->transforms.size() << "Map_ID.size = : " << map_targets_.size() << "Map_MEAS.size = : " << map_measured_pose_.size() << std::endl;
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
    if(  ( map_targets_.size() != 0 ) )
    {
      for(auto& it_targets : map_targets_)
      {
        unsigned int target_id = map_targets_[it_targets.first].id;
        Eigen::Vector7d meas_pose = map_targets_[it_targets.first].measured_pose_;

        if(map_targets_[it_targets.first].new_meas_)
        {
          // CHECKME if necessary -> the check is already done above
          // check target existence -> if not, create new target
          if( manager_.getTarget(target_id)==nullptr )
          {
            // init the target
            double dt0 = dt_;
            double t0 = 0.0;
            manager_.init(target_id,dt0,Q_,R_,P_,meas_pose,t0,type_);
          }
          // CHECKME if necessary

          // update with measurements
          manager_.update(target_id,dt,meas_pose);
          map_targets_[it_targets.first].new_meas_ = false;
        }
        else
        {
          // update without measurements (only prediction)
          manager_.update(target_id,dt);
        }

        // Assign estimated values to the map of targets
        map_targets_[it_targets.first].estimated_pose_ = manager_.getTarget(target_id)->getEstimatedPose();
        map_targets_[it_targets.first].estimted_twist_ = manager_.getTarget(target_id)->getEstimatedTwist();
        map_targets_[it_targets.first].estimated_quaternion_ = manager_.getTarget(target_id)->getEstimatedOrientation();
        map_targets_[it_targets.first].estimated_rpy_ = manager_.getTarget(target_id)->getEstimatedRPY();
        map_targets_[it_targets.first].estimated_position_ = manager_.getTarget(target_id)->getEstimatedPosition();

      } // end for

    } // end if

    if( ( map_targets_.size() != 0 ) )
    {
      for(auto it_targets : map_targets_)
      {
        target_id = map_targets_[it_targets.first].id;

        // Get the interception point
        Eigen::Vector7d inteception_pose;
        if( manager_.getInterceptionPose(target_id, t_, pos_th_, ang_th_, inteception_pose) )
        {
          map_targets_[it_targets.first].intercepted_ = true;
          map_targets_[it_targets.first].interception_pose_ = inteception_pose;

#ifdef DEBUG_tmp
        std::cout << "Interception pose" << std::endl;
        std::cout << inteception_pose << std::endl;
#endif
        }
        else
        {
          map_targets_[it_targets.first].intercepted_ = false;
        }

        Eigen::Vector3d target_position;
        Eigen::Quaterniond target_orientation;
        target_position = map_targets_[it_targets.first].estimated_position_;
        target_orientation = map_targets_[it_targets.first].estimated_quaternion_;
        std::string target_name = it_targets.first;

        // Publish to tf wrt world frame
        sendTF(target_position, target_orientation, target_name, camera_frame_, transform_, q_, br_);

        if(publish_to_robot_)
        {
          // start tracking after first interception occurred
          poseToStampedPose(target_position, target_orientation, franka_eq_pose_msg_, target_name, count);
          franka_eq_pose_pub_.publish(franka_eq_pose_msg_);
        }

      } // end for

    } // end if

    // update the time-related variables
    t_= t_ + dt;
    t_prev_ = t;

    meas_lock.unlock();
  }
}

void RosTargetManager::setWorldFrameName(std::string& name)
{
  world_name_frame_ = name;
}

void RosTargetManager::setTargetFrameToken(std::string& token)
{
  target_name_frame_ = token;
}

void RosTargetManager::setCameraFrame(std::string& camera_frame)
{
  camera_frame_ = camera_frame;
}

void RosTargetManager::sendTF(Eigen::Vector3d &postion, Eigen::Quaterniond &orientation, std::string &target_name, std::string &world_name, tf::Transform &transform, tf::Quaternion &q, tf::TransformBroadcaster &br)
{
  transform.setOrigin(tf::Vector3(postion.x(),postion.y(),postion.z()));

  q = tf::Quaternion(orientation.x(),orientation.y(),orientation.z(),orientation.w());
  q.normalize();
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), (world_name), (target_name + "_est") ));
}

void RosTargetManager::poseToStampedPose(Eigen::Vector3d &position, Eigen::Quaterniond &orientation,
                                         geometry_msgs::PoseStamped &eq_pose_pose_stamped_msg, std::string child_frame_name, const unsigned int &count)
{
  eq_pose_pose_stamped_msg.header.frame_id    = child_frame_name;
  eq_pose_pose_stamped_msg.header.seq         = count;
  eq_pose_pose_stamped_msg.header.stamp       = ros::Time::now();

  eq_pose_pose_stamped_msg.pose.position.x    = position.x();
  eq_pose_pose_stamped_msg.pose.position.y    = position.y();
  eq_pose_pose_stamped_msg.pose.position.z    = position.z();
  eq_pose_pose_stamped_msg.pose.orientation.x = orientation.x();
  eq_pose_pose_stamped_msg.pose.orientation.y = orientation.y();
  eq_pose_pose_stamped_msg.pose.orientation.z = orientation.z();
  eq_pose_pose_stamped_msg.pose.orientation.w = orientation.w();
}

void RosTargetManager::poseToStampedPose(Eigen::Vector7d &pose, geometry_msgs::PoseStamped &eq_pose_pose_stamped_msg,
                                         std::string child_frame_name, const unsigned int &count)
{

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  posetoPositionQuat(pose, position, orientation);

  eq_pose_pose_stamped_msg.header.frame_id    = child_frame_name;
  eq_pose_pose_stamped_msg.header.seq         = count;
  eq_pose_pose_stamped_msg.header.stamp       = ros::Time::now();

  eq_pose_pose_stamped_msg.pose.position.x    = position.x();
  eq_pose_pose_stamped_msg.pose.position.y    = position.y();
  eq_pose_pose_stamped_msg.pose.position.z    = position.z();
  eq_pose_pose_stamped_msg.pose.orientation.x = orientation.x();
  eq_pose_pose_stamped_msg.pose.orientation.y = orientation.y();
  eq_pose_pose_stamped_msg.pose.orientation.z = orientation.z();
  eq_pose_pose_stamped_msg.pose.orientation.w = orientation.w();
}

void RosTargetManager::setRobotTopicEqPose(std::string& topic_name)
{
  robot_topic_ = topic_name;
}
