#include "target_estimation/intersection_solver.hpp"


double Solver::lowestRealRoot(const Eigen::VectorXd &coeffs) {

  if (std::abs(coeffs(coeffs.size()-1))>0.0)
    solver.compute(coeffs);
  else
    return -1;

  bool reRootExists = false;
  double imThreshold = 1e-10; //threshold for saying that the imaginary part is a rounding error
  auto r = solver.smallestRealRoot(reRootExists, imThreshold);

  if(!reRootExists) return -1;
  return r;
}

IntersectionSolver::IntersectionSolver(TargetManager::Ptr target_manager, const unsigned int filters_length)
{
  assert(target_manager);
  target_manager_ = target_manager;
  // filters_length = filter_time/dt
  // Example filters_length = 250 means that with 50hz loop we have 5.0 secs of filter
  pos_error_filter_.reset(new MovingAvgFilter(filters_length));
  ang_error_filter_.reset(new MovingAvgFilter(filters_length));

  // 5 coefficients for a 2rd order system
  coeff_.resize(5);
  coeff_.setZero();

  // Reset tmp variables
  q1_tmp_.normalize();
  q2_tmp_.normalize();
  pos_tmp_.setZero();
  vel_tmp_.setZero();
  acc_tmp_.setZero();

  initPose(intersection_pose_prev_);
}

double IntersectionSolver::getIntersectionTimeWithSphere(const unsigned int& id, const double& t1, const Eigen::Vector3d& origin, const double& radius)
{

  if(target_manager_->getTarget(id))
  {
    pos_tmp_ = target_manager_->getTarget(id)->getEstimatedPose(t1).head(3);
    vel_tmp_ = target_manager_->getTarget(id)->getEstimatedTwist(t1).head(3);
    acc_tmp_ = target_manager_->getTarget(id)->getEstimatedAcceleration(t1).head(3);

    // Use only the translational part of the variables
    double x = pos_tmp_(0) - origin(0);
    double y = pos_tmp_(1) - origin(1);
    double z = pos_tmp_(2) - origin(2);
    double vx = vel_tmp_(0);
    double vy = vel_tmp_(1);
    double vz = vel_tmp_(2);
    double ax = acc_tmp_(0);
    double ay = acc_tmp_(1);
    double az = acc_tmp_(2);
    double R = radius;

    //if(acceleration_on_)
    //{
    // 2rd order system
    coeff_(4) = 0.25 * ax*ax + ay*ay + az*az;
    coeff_(3) = vx*ax + vy*ay + vz*az;
    coeff_(2) = vx*vx + vy*vy + vz*vz + x*ax + y*ay + z*az;
    coeff_(1) = 2*(x*vx + y*vy + z*vz);
    coeff_(0) = x*x + y*y + z*z - R*R;
    //}
    //else
    //{
    //  // 1st order system
    //  coeff.resize(3);
    //  coeff(2) = vx*vx + vy*vy + vz*vz;
    //  coeff(1) = 2*(x*vx + y*vy + z*vz);
    //  coeff(0) = x*x + y*y + z*z - R*R;
    //}

    double delta_intersect_t = solver_.lowestRealRoot(coeff_);

    if(delta_intersect_t < 0) return -1;

    return delta_intersect_t;
  }
  else
    return -1;
}

bool IntersectionSolver::getIntersectionPoseWithSphere(const unsigned int& id, const double& t1, const double& pos_th, const double& ang_th,
                                                       const Eigen::Vector3d& origin, const double& radius, Eigen::Vector7d& intersection_pose)
{
  assert(t1>=0.0);
  assert(pos_th>=0.0);
  assert(ang_th>=0.0);
  double delta_intersect_t = -1;
  bool converged = false;
  initPose(intersection_pose);

  delta_intersect_t = getIntersectionTimeWithSphere(id,t1,origin,radius);
  if (delta_intersect_t > -1)
  {
    Eigen::Quaterniond q1, q2;
    intersection_pose = target_manager_->getTarget(id)->getEstimatedPose(delta_intersect_t+t1);
    double pos_error = (POSE_pos(intersection_pose) - POSE_pos(intersection_pose_prev_)).norm();
    q1.coeffs() = POSE_quat(intersection_pose);
    q2.coeffs() = POSE_quat(intersection_pose_prev_);
    q1.normalize();
    q2.normalize();
    double ang_error = std::abs(wrapMinMax(computeQuaternionErrorAngle(q1,q2), -M_PI, M_PI));

    // Check if errors converged  using a moving average filter
    double pos_error_filt = pos_error_filter_->update(pos_error);
    double ang_error_filt = ang_error_filter_->update(ang_error);

    // Save the value
    intersection_pose_prev_ = intersection_pose;

    if(pos_error_filt <= pos_th && ang_error_filt <= ang_th)
      converged = true;

  }
  return converged;
}

bool IntersectionSolver::getIntersectionPoseAndtimeWithSphere(const unsigned int& id, const double& t1, const double& pos_th, const double& ang_th,
                                          const Eigen::Vector3d& origin, const double& radius,
                                          Eigen::Vector7d& intersection_pose, double& delta_intersect_t)
{
  assert(t1>=0.0);
  assert(pos_th>=0.0);
  assert(ang_th>=0.0);
  bool converged = false;
  initPose(intersection_pose);

  delta_intersect_t = getIntersectionTimeWithSphere(id,t1,origin,radius);
  if (delta_intersect_t > -1)
  {
    Eigen::Quaterniond q1, q2;
    intersection_pose = target_manager_->getTarget(id)->getEstimatedPose(delta_intersect_t+t1);

    double pos_error = (POSE_pos(intersection_pose) - POSE_pos(intersection_pose_prev_)).norm();
    q1_tmp_.coeffs() = POSE_quat(intersection_pose);
    q2_tmp_.coeffs() = POSE_quat(intersection_pose_prev_);
    q1_tmp_.normalize();
    q2_tmp_.normalize();
    double ang_error = std::abs(wrapMinMax(computeQuaternionErrorAngle(q1_tmp_,q2_tmp_), -M_PI, M_PI));

    // Check if errors converged  using a moving average filter
    double pos_error_filt = pos_error_filter_->update(pos_error);
    double ang_error_filt = ang_error_filter_->update(ang_error);

    // Save the value
    intersection_pose_prev_ = intersection_pose;

    if(pos_error_filt <= pos_th && ang_error_filt <= ang_th)
      converged = true;

  }
  return converged;
}
