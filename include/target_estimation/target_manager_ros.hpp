#ifndef TARGET_MANAGER_ROS_HPP
#define TARGET_MANAGER_ROS_HPP

#include <iostream>
#include <iomanip>
#include <vector>

#include <target_estimation/TargetEstimation.h>
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

class RosTargetManager
{

public:

  RosTargetManager(ros::NodeHandle& nh);

  void setInterceptionSphere(const Eigen::Vector3d& pos, const double& radius);

  void update(const double& dt);

  void setTargetTokenName(const std::string& token_name);

  void setReferenceFrameName(const std::string& frame);

  void setPositionConvergenceThreshold(const double& th);

  void setAngularConvergenceThreshold(const double& th);

  const Eigen::Vector7d& getInterceptionPose() const;

  void calculateInterceptionPose(bool active);

  void setExpirationTime(double time);

private:

  struct Measurement
  {
    geometry_msgs::TransformStamped tr_;
    bool new_meas_ = true;
    double last_meas_time  = 0.0;
  };

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

  Eigen::Vector7d interception_pose_;

  double t_;
  double pos_th_;
  double ang_th_;

  std::mutex meas_lock_;

  meas_map_t measurements_;

  Eigen::Vector7d tmp_vector7d_;
  Eigen::Isometry3d tmp_isometry3d_;
  tf::Transform tmp_transform_;

  tf::TransformBroadcaster br_;

  ros::Time ros_t_;

  std::atomic<bool> interception_pose_on_;
  std::atomic<double> expiration_time_;
};

#endif
