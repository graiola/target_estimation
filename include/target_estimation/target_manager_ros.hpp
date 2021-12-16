#ifndef TARGET_MANAGER_ROS_HPP
#define TARGET_MANAGER_ROS_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <atomic>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include "target_estimation/target_manager.hpp"

inline void transformStampedToPose7d(const geometry_msgs::TransformStamped& t, Eigen::Vector7d& e)
{
  e(0) = t.transform.translation.x;
  e(1) = t.transform.translation.y;
  e(2) = t.transform.translation.z;

  e(3) = t.transform.rotation.x;
  e(4) = t.transform.rotation.y;
  e(5) = t.transform.rotation.z;
  e(6) = t.transform.rotation.w;
}

inline void pose7dToTFTransform(const Eigen::Vector7d& e, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(e(0),e(1),e(2)));
  t.setRotation(tf::Quaternion(e(3),e(4),e(5),e(6)));
}

inline void isometryToTFTransform(const Eigen::Isometry3d& e, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(e.translation().x(),e.translation().y(),e.translation().z()));
  Eigen::Quaterniond q;
  rotToQuat(e.rotation(),q);
  t.setRotation(tf::Quaternion(q.coeffs().x(),q.coeffs().y(),q.coeffs().z(),q.coeffs().w())); // FIXME check if it works
}

inline void pose7dToTransformStamped(const Eigen::Vector7d& e, geometry_msgs::TransformStamped& t)
{
  t.transform.translation.x = e(0);
  t.transform.translation.y = e(1);
  t.transform.translation.z = e(2);

  t.transform.rotation.x = e(3);
  t.transform.rotation.y = e(4);
  t.transform.rotation.z = e(5);
  t.transform.rotation.w = e(6);
}

class Measurement
{
public:

  Measurement()
  {
    new_meas_ = true;
    last_meas_time_  = 0.0;
  }

  inline bool read(geometry_msgs::TransformStamped& tr)
  {
    bool read_successful = false;
    if(new_meas_ && meas_lock_.try_lock())
    {
      tr = tr_;
      read_successful = true;
      meas_lock_.unlock();
    }
    return read_successful;
  }

  inline void update(const geometry_msgs::TransformStamped& tr)
  {
    double current_time_stamp = 0.0;
    double prev_time_stamp = 0.0;
    meas_lock_.lock();
    current_time_stamp = toSec(tr.header.stamp.sec,tr.header.stamp.nsec);
    prev_time_stamp = toSec(tr_.header.stamp.sec,tr_.header.stamp.nsec);
    if(current_time_stamp > prev_time_stamp) // New measurement
    {
      new_meas_ = true;
      last_meas_time_ = current_time_stamp;
    }
    else
    {
      new_meas_ = false; // No new measurement
    }
    tr_ = tr; // Save the measurement w.r.t observer
    meas_lock_.unlock();
  }

  double getTime() const
  {
    return last_meas_time_;
  }

  std::string getFrameId()
  {
    return tr_.header.frame_id;
  }

private:

  std::atomic<double> last_meas_time_;
  std::atomic<bool> new_meas_;
  geometry_msgs::TransformStamped tr_;
  std::mutex meas_lock_;
};

class RosTargetManager
{

public:

  RosTargetManager(ros::NodeHandle& nh);

  void update(const double& dt);

  void setTargetTokenName(const std::string& token_name);

  void setExpirationTime(double time);

private:

  typedef std::map<unsigned int, Measurement> meas_map_t;

  void measurementCallBack(const tf2_msgs::TFMessage::ConstPtr& pose_msg);
  bool parseSquareMatrix(const ros::NodeHandle& n, const std::string& matrix, Eigen::MatrixXd& M);
  bool parseTargetType(const ros::NodeHandle& n, TargetManager::target_t& type);

  ros::NodeHandle nh_;
  ros::Subscriber meas_subscriber_;

  TargetManager manager_;
  TargetManager::target_t type_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd R_;

  std::string token_name_;

  double t_;

  meas_map_t measurements_;

  Eigen::Vector7d tmp_vector7d_;
  Eigen::Isometry3d tmp_isometry3d_;
  tf::Transform tmp_tf_tr_;

  tf::TransformBroadcaster br_;
  geometry_msgs::TransformStamped tmp_tr_;

  ros::Time ros_t_;

  std::atomic<double> expiration_time_;
};

#endif
