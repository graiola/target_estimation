#include "target_estimation/target_manager_ros.hpp"

#include <stdexcept>

using namespace std;

//#define DEBUG
#define DEBUG_tmp

RosTargetManager::RosTargetManager(ros::NodeHandle& nh, double& dt, std::string& topic_to_publish, tf::TransformBroadcaster& br)
{
  interface_with_robot_ = false;

  init(nh, dt, br, topic_to_publish, interface_with_robot_);
}

RosTargetManager::RosTargetManager(ros::NodeHandle& nh, double& dt, tf::TransformBroadcaster& br, std::string& topic_to_publish, std::string& robot_topic_to_publish)
{
  interface_with_robot_ = true;
  setRobotTopicEqPose(robot_topic_to_publish);

  init(nh, dt, br, topic_to_publish, interface_with_robot_);
}

void RosTargetManager::init(ros::NodeHandle& nh, double& dt, tf::TransformBroadcaster& br, std::string& topic_to_publish, const bool &publish_robot)
{

  // subscribe to topic_to_publish
  tf_topic_name_ = topic_to_publish;
  nh_ = nh;
  measurament_sub_ = nh_.subscribe(topic_to_publish, 1 , &RosTargetManager::MeasurementCallBack, this);

  br_ = br;

  // FIXME: chang name of the bool to interface_with_robot
  if(publish_robot)
  {
    robot_eq_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(robot_topic_pub_, 1, true);
  }

  n_active_frames_ = 0;

  t_ = 0.0;
  dt_ = dt;
  t_prev_ = 0.0;
  // FIXME -> adjust thresholds
  pos_th_ = 0.001; // [m]
  ang_th_ = 0.001; // [rad]
  // FIXME -> adjust thresholds

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
}

void RosTargetManager::setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius)
{
  manager_.setInterceptionSphere(pos, radius);
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
    // 3- read data from each target
    Eigen::Vector7d meas_pose;
    meas_pose(0) = pose_msg->transforms.data()->transform.translation.x;
    meas_pose(1) = pose_msg->transforms.data()->transform.translation.y;
    meas_pose(2) = pose_msg->transforms.data()->transform.translation.z;
    meas_pose(3) = pose_msg->transforms.data()->transform.rotation.x;
    meas_pose(4) = pose_msg->transforms.data()->transform.rotation.y;
    meas_pose(5) = pose_msg->transforms.data()->transform.rotation.z;
    meas_pose(6) = pose_msg->transforms.data()->transform.rotation.w;

    unsigned int target_id;
    if(map_targets_.count(current_frame) != 1)
    {
      // target not found, so add it!
      if(getId(current_frame, target_id))
      {
        map_targets_[current_frame].id = target_id;
        n_active_frames_ ++;
      }
      else
      {
        map_targets_[current_frame].id = n_active_frames_;
        n_active_frames_ ++;
      }

      // check target existence of a KF applied to the target -> if not, create new KF
      if( manager_.getTarget(map_targets_[current_frame].id) == nullptr )
      {
        double dt0 = dt_;
        double t0 = 0.0;
        manager_.init(map_targets_[current_frame].id, dt0, Q_, R_, P_, meas_pose, t0, type_);
      }
    }

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

void RosTargetManager::update(const double& dt, const unsigned int &count)
{
  if(meas_lock.try_lock())
  {
    double t = ros::Time::now().toSec();
    t_ = t;
    dt_ = t - t_prev_;

    if( map_targets_.size() > 0 )
    {
      for(const auto& it_targets : map_targets_)
      {
        unsigned int target_id = map_targets_[it_targets.first].id;
        Eigen::Vector7d meas_pose = map_targets_[it_targets.first].measured_pose_;

        if(map_targets_[it_targets.first].new_meas_)
        {
          // update with measurements
          manager_.update(target_id,dt,meas_pose);
          map_targets_[it_targets.first].new_meas_ = false;
        }
        else
        {
          // update without measurements (only prediction)
          manager_.update(target_id, dt);
        }

        // Assign estimated values to the map of targets
        Eigen::Vector7d est_pose;
        est_pose = manager_.getTarget(target_id)->getEstimatedPose();

  #ifdef DEBUG
          std::cout << "Target Name: " << map_targets_[it_targets.first].frame_name_ << std::endl;
  #endif

        // Get the interception point
        Eigen::Vector7d inteception_pose;
        if( manager_.getInterceptionPose(target_id, t_, pos_th_, ang_th_, inteception_pose) )
        {
          map_targets_[it_targets.first].converged_ = true;

  #ifdef DEBUG
          std::cout << "Interception pose" << std::endl;
          std::cout << inteception_pose << std::endl;
  #endif
        }
        else
        {
          map_targets_[it_targets.first].converged_ = false;
        }


#ifdef DEBUG
        std::cout << "Estimated pose " << est_pose << std::endl;
#endif

        std::string target_frame = map_targets_[it_targets.first].frame_name_;

        // Publish to tf wrt world frame
        sendTF(est_pose, target_frame, world_frame_, transform_, q_, br_);

        Eigen::Vector3d target_position;
        Eigen::Quaterniond target_orientation;
        if( interface_with_robot_ )
        {

          if( map_targets_[it_targets.first].converged_ )
          {
#ifdef DEBUG
            std::cout << "Interception pose " << std::endl;
            std::cout << inteception_pose << std::endl;
#endif
            posetoPositionQuat(inteception_pose, target_position, target_orientation);

            // start tracking after first interception has occurred
            poseToStampedPose(target_position, target_orientation, robot_eq_pose_msg_, target_frame, count);
            robot_eq_pose_pub_.publish(robot_eq_pose_msg_);

            map_targets_[it_targets.first].intercepted_ = true; // TESTME: check whether it is better to compute the error between the target pose and the actual one to set this parameter to true;
          } // end if

          if(map_targets_[it_targets.first].intercepted_)
          {
            posetoPositionQuat(est_pose, target_position, target_orientation);

            // start tracking after first interception has occurred
            poseToStampedPose(target_position, target_orientation, robot_eq_pose_msg_, target_frame, count);
            robot_eq_pose_pub_.publish(robot_eq_pose_msg_);
          } // end if

        } // end if

      } // end for

    }
    // update the time-related variables
    t_= t_ + dt_;
    t_prev_ = t;
    meas_lock.unlock();
  }
}

void RosTargetManager::sendTF(Eigen::Vector7d &pose, std::string &target_name, std::string &world_name, tf::Transform &transform, tf::Quaternion &q, tf::TransformBroadcaster &br)
{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  posetoPositionQuat(pose, position, orientation);

  transform.setOrigin( tf::Vector3( position.x(),position.y(),position.z() ) );

  q = tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w());
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

void RosTargetManager::setWorldFrameName(std::string& name)
{
  world_frame_ = name;
}

void RosTargetManager::setTargetFrameToken(std::string& token)
{
  target_name_frame_ = token;
}

void RosTargetManager::setCameraFrame(std::string& camera_frame)
{
  camera_frame_ = camera_frame;
}

void RosTargetManager::setRobotTopicEqPose(std::string& topic_name)
{
  robot_topic_pub_ = topic_name;
}
