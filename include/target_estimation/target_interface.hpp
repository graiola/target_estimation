/**
*
*/

#ifndef TARGET_INTERFACE_HPP
#define TARGET_INTERFACE_HPP

#include <map>
#include <string>
#include <fstream>
#include <mutex>
#include <iostream>
#include <Eigen/Dense>
#include <memory>

#include "target_estimation/kalman.hpp"
#include "target_estimation/solver.hpp"
#include "target_estimation/utils.hpp"
#include "target_estimation/geometry.hpp"

/**
 * @brief The TargetInterface class
 * Generic interface for the different implementations.
 */
class TargetInterface
{

public:

  typedef std::shared_ptr<TargetInterface> Ptr;

  TargetInterface(const unsigned int& id, const Eigen::MatrixXd P0, const double& t0);

  TargetInterface() : id_(-1) {};

  /**
     * @brief addMeasurement Add a measured value to the estimator and perform an update step
     * of the filters
     * @param dt
     * @param meas
     */
  virtual void addMeasurement(const double& dt, const Eigen::Vector7d& meas) = 0;


  /**
    * @brief update Perform a predict step of the filters when no measurement given
    * @param dt
    */
  virtual void update(const double& dt) = 0;

  /**
     * @brief getEstimatedPose
     * @param t
     * @return Target's estimated pose at time t [x y z qx qy qz qw]
     */
  virtual Eigen::Vector7d getEstimatedPose(const double& t);

  /**
     * @brief getEstimatedTwist
     * @param t
     * @return Target's estimated twist at time t
     */
  virtual Eigen::Vector6d getEstimatedTwist(const double& t);

  /**
     * @brief getEstimatedAcceleration
     * @param t
     * @return Target's estimated acceleration at time t
     */
  virtual Eigen::Vector6d getEstimatedAcceleration(const double& t);

  /**
     * @brief getIntersectionTime
     * @param t
     * @param origin
     * @param radius
     * @return the time the Target will first intersect with a sphere at the given origin and radius, or -1 if it will not happen.
     */
  double getIntersectionTime(const double& t, const Eigen::Vector3d& origin, const double& radius);

  /**
     * @brief getIntersectionPose
     * @param t
     * @param pos_th maximum allowable position error [m]
     * @param ang_th maximum allowable angular error [rad]
     * @param origin
     * @param radius
     * @param intersection_pose
     * @return true if an intersection pose with a sphere at the given origin and radius exists, false otherwise.
     */
  bool getIntersectionPose(const double& t, const double& pos_th, const double& ang_th, const Eigen::Vector3d& origin, const double& radius, Eigen::Vector7d& intersection_pose);

  /**
     * @brief getPeriodEstimate
     * @return estimated rotation period
     */
  double getPeriodEstimate();

  /**
     * @brief getTime
     * @return internal time
     */
  double getTime();

  /**
     * @brief getEstimatedPose
     * @return last estimated Target's pose
     */
  const Eigen::Vector7d& getEstimatedPose();

  /**
     * @brief getEstimatedTransform
     * @return last estimated Target's pose
     */
  const Eigen::Isometry3d& getEstimatedTransform();

  /**
     * @brief getEstimatedPosition
     * @return last estimated Target's position (x,y,z)
     */
  const Eigen::Vector3d& getEstimatedPosition();

  /**
     * @brief getEstimatedOrientation
     * @return last estimated Target's orientation (quaternion)
     */
  const Eigen::Quaterniond& getEstimatedOrientation();

  /**
     * @brief getEstimatedRPY
     * @return last estimated Target's orientation (rpy)
     */
  const Eigen::Vector3d& getEstimatedRPY();

  /**
     * @brief getEstimatedTwist
     * @return last estimated Target's linear velocity and angular velocity
     */
  const Eigen::Vector6d& getEstimatedTwist();

  /**
     * @brief getEstimatedAcceleration
     * @return last estimated Target's acceleration
     */
  const Eigen::Vector6d& getEstimatedAcceleration();

  /**
     * @brief getIntersectionPose
     * @return last computed Target's intersection pose
     */
  const Eigen::Vector7d& getIntersectionPose();

  /**
     * @brief getMeasuredPose
     * @return last measured Target's pose
     */
  const Eigen::Vector7d& getMeasuredPose();

  /**
     * @brief getID
     * @return Target's ID
     */
  unsigned int getID() const {return id_;}

  /**
     * @brief getN
     * @return n state dimension
     */
  unsigned int getN() const {return n_;}

  /**
     * @brief getM
     * @return m measurement dimension
     */
  unsigned int getM() const {return m_;}

  /**
     * @brief getEstimator
     * @return get a constant reference to the kalman filter
     */
  KalmanFilterInterface* getEstimator() const {return estimator_.get();}

  /**
     * @brief getNumberMeasurements
     * @return the number of measurements done so far
     */
  long long getNumberMeasurements() const {return n_meas_;}

  /**
     * @brief log Log data through ros
     */
  void log();

protected:

  /**
     * @brief updateTargetState Update the internal variables representing
     * the state of the target
     */
  virtual void updateTargetState() = 0;

  /**
     * @brief updateMeasurement Update the measurement and the measurement counter
     */
  void updateMeasurement(const Eigen::Vector7d& meas);

  /**
     * @brief updateTime  Update internal time variable
     */
  void updateTime(const double& dt);

  /**
     * @brief n_ Number of states
     */
  unsigned int n_;

  /**
     * @brief m_ Number of measurements
     */
  unsigned int m_;

  /**
     * @brief id_ ID number
     */
  int id_;

  /**
     * @brief t_ Time of last estimate
     */
  double t_;

  /**
     * @brief pose_ Estimated pose transform
     */
  Eigen::Isometry3d T_;

  /**
     * @brief pose_ Estimated pose
     */
  Eigen::Vector7d pose_;

  /**
     * @brief position_ Estimated position
     */
  Eigen::Vector3d position_;

  /**
     * @brief q_ Estimated orientation, represented as quaternion
     */
  Eigen::Quaterniond q_;

  /**
     * @brief q_ Estimated orientation, represented as roll pitch and yaw angles
     */
  Eigen::Vector3d rpy_;

  /**
     * @brief twist_ Estimated twist
     */
  Eigen::Vector6d twist_;

  /**
     * @brief acceleration_ Estimated acceleration
     */
  Eigen::Vector6d acceleration_;

  /**
     * @brief intersection_pose_ Computed insercetion pose with the sphere
     */
  Eigen::Vector7d intersection_pose_;

  /**
     * @brief measured_pose_ Measured pose
     */
  Eigen::Vector7d measured_pose_;

  /**
     * @brief P_ Kalman Filter covariance matrix
     */
  Eigen::MatrixXd P_;

  /**
     * @brief x_ Estimated state, with vectors of pose/twist/acceleration
     */
  Eigen::VectorXd x_;

  /**
     * @brief estimator_ State estimator
     */
  KalmanFilterInterface::Ptr estimator_;

  /**
     * @brief n_meas_ How many measurements we've made
     */
  long long n_meas_;

  /**
     * @brief data_lock_ Used to prevent concurrent access to variables
     */
  std::mutex data_lock_;

  /**
     * @brief solver_ Polynomial solver
     */
  Solver solver_;

  /**
     * @brief Moving average filters
     */
  MovingAvgFilter::Ptr pos_error_filter_;
  MovingAvgFilter::Ptr ang_error_filter_;

  /**
     * @brief C_ output matrix
     */
  Eigen::MatrixXd C_;

  /**
     * @brief A_ dynamical system matrix
     */
  Eigen::MatrixXd A_;

  /**
     * @brief acceleration_on_ true if the acceleration is integrated in the dynamical system
     */
  bool acceleration_on_;

  /**
     * @brief Temporary variables
     */
  Eigen::Vector3d vector3d_tmp_;
  Eigen::Vector6d vector6d_tmp_;
  Eigen::Vector7d vector7d_tmp_;
  Eigen::Isometry3d isometry3d_tmp_;
  Eigen::Quaterniond quaterniond_tmp_;
  Eigen::Vector3d pos_tmp_;
  Eigen::Vector3d vel_tmp_;
  Eigen::Vector3d acc_tmp_;
};

#endif
