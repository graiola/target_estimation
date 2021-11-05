#include <gtest/gtest.h>
#include <ros/ros.h>
#include <vector>
#include <random>
#include <Eigen/Dense>

#include "target_estimation/target_manager.hpp"

static TargetManager _manager;
// Simulated measurement noise
const double _pObserved_stddev = 0.001; // 1mm
const double _pObserved_mean = 0.0;
// Statistical generators
static std::default_random_engine _generator;
static std::normal_distribution<double> normal_dist(_pObserved_mean, _pObserved_stddev);
static double _dt = 0.01;
static unsigned int _n_points = 10000;
static double _end_goal_x = 0.2;
static double _end_goal_y = 0.3;
static double _end_goal_z = 0.4;
static Eigen::Vector3d _omega(0.0, 0.1, 0.0);

void generateMatrices(Eigen::MatrixXd& Q, Eigen::MatrixXd& R, Eigen::MatrixXd& P)
{
  Q.resize(12,12);
  R.resize(6,6);
  P.resize(12,12);

  Q      << 2.5000e-17,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-14,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   2.5000e-17,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-14,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   2.5000e-17,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-14,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   2.5000e-19,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-16,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   2.5000e-19,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-16,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   2.5000e-19,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-16,
            5.0000e-14,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-10,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   5.0000e-14,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-10,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   5.0000e-14,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-10,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-16,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-12,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-16,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-12,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-16,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-12;

  R      << 1.0000e-04,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   1.0000e-04,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   1.0000e-04,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-04,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-04,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-04;

  P      << 1.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,
            0.00000,   1.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,
            0.00000,   0.00000,   1.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,
            0.00000,   0.00000,   0.00000,   1.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,
            0.00000,   0.00000,   0.00000,   0.00000,   1.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,
            0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   1.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,
            0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.01000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,
            0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.01000,   0.00000,   0.00000,   0.00000,   0.00000,
            0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.01000,   0.00000,   0.00000,   0.00000,
            0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.01000,   0.00000,   0.00000,
            0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.01000,   0.00000,
            0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.01000;
}

void generateMeasurements(const double dt, const unsigned int n_points,
                          const double end_goal_x, const double end_goal_y, const double end_goal_z,
                          const Eigen::Vector3d omega,
                          Eigen::MatrixXd& meas_pose, Eigen::MatrixXd& pose, Eigen::VectorXd& time)
{
  // List of position measurements x,y,z and quaternion
  // Generate a linear motion on x y z and an angular motion using omega
  time      = Eigen::VectorXd::Zero(n_points);
  meas_pose = Eigen::MatrixXd::Zero(n_points,7);
  pose      = Eigen::MatrixXd::Zero(n_points,7);
  pose.col(0) = Eigen::VectorXd::LinSpaced(n_points, 0.0, end_goal_x); // Move on the x
  pose.col(1) = Eigen::VectorXd::LinSpaced(n_points, 0.0, end_goal_y); // Move on the y
  pose.col(2) = Eigen::VectorXd::LinSpaced(n_points, 0.0, end_goal_z); // Move on the z
  Eigen::Quaterniond q_meas = Eigen::Quaterniond::Identity();

  double t = 0.0;

  // Update the measurement
  for(unsigned int i=0;i<n_points;i++)
  {
      meas_pose(i,0) = pose(i,0) + normal_dist(_generator);
      meas_pose(i,1) = pose(i,1) + normal_dist(_generator);
      meas_pose(i,2) = pose(i,2) + normal_dist(_generator);

      meas_pose(i,3) = pose(i,3) = q_meas.x();
      meas_pose(i,4) = pose(i,4) = q_meas.y();
      meas_pose(i,5) = pose(i,5) = q_meas.z();
      meas_pose(i,6) = pose(i,6) = q_meas.w();

      // Calculate next orientation
      q_meas.coeffs() = Qtran(dt,omega) * q_meas.coeffs();
      q_meas.normalize();

      time(i) = t += dt;
  }
}

Eigen::Vector3d calculateVelocities()
{
  double total_time = _n_points * _dt;
  Eigen::Vector3d velocities;
  velocities << _end_goal_x, _end_goal_y, _end_goal_z;
  return velocities/total_time;
}

TEST(test_target, RPY )
{

    Eigen::MatrixXd Q,R,P;
    generateMatrices(Q,R,P);
    Eigen::MatrixXd pose, meas_pose;
    Eigen::VectorXd time;
    generateMeasurements(_dt,_n_points,_end_goal_x,_end_goal_y,_end_goal_z,_omega,meas_pose,pose,time);

    Eigen::Vector7d p0;
    initPose(p0);
    _manager.init(0,_dt,Q,R,P,p0,0.0,TargetManager::target_t::RPY);
    _manager.init(1,_dt,Q,R,P,p0,0.0,TargetManager::target_t::RPY_EXT);

    // Feed measurements into filter, output estimated states
    Eigen::MatrixXd rpy_pose_est(_n_points,7);
    Eigen::MatrixXd rpy_twist_est(_n_points,6);
    Eigen::MatrixXd rpy_ext_pose_est(_n_points,7);
    Eigen::MatrixXd rpy_ext_twist_est(_n_points,6);
    Eigen::MatrixXd quat_pose_est(_n_points,7);
    Eigen::MatrixXd quat_twist_est(_n_points,6);
    Eigen::MatrixXd rpy_sigma_xyz(_n_points,3);
    Eigen::MatrixXd rpy_ext_sigma_xyz(_n_points,3);

    Eigen::Vector7d current_meas;
    Eigen::Vector7d current_pose_est;
    Eigen::Vector6d current_twist_est;

    for(unsigned int i = 0; i < _n_points; i++) {

        current_meas = meas_pose.row(i);

        _manager.update(0,_dt,current_meas);
        _manager.update(1,_dt,current_meas);

        _manager.getTargetPose(0,current_pose_est);
        rpy_pose_est.row(i) = current_pose_est;
        _manager.getTargetTwist(0,current_twist_est);
        rpy_twist_est.row(i) = current_twist_est;

        _manager.getTargetPose(1,current_pose_est);
        rpy_ext_pose_est.row(i) = current_pose_est;
        _manager.getTargetTwist(1,current_twist_est);
        rpy_ext_twist_est.row(i) = current_twist_est;

        rpy_sigma_xyz = _manager.getTarget(0)->getEstimator()->getP().topLeftCorner(3,3).diagonal();
        rpy_ext_sigma_xyz = _manager.getTarget(1)->getEstimator()->getP().topLeftCorner(3,3).diagonal();
    }

    writeTxtFile("time",time);
    writeTxtFile("pose",pose);
    writeTxtFile("meas_pose",meas_pose);
    writeTxtFile("rpy_est_pose",rpy_pose_est);
    writeTxtFile("rpy_est_twist",rpy_twist_est);
    writeTxtFile("rpy_ext_est_pose",rpy_ext_pose_est);
    writeTxtFile("rpy_ext_est_twist",rpy_ext_twist_est);

    auto velocities = calculateVelocities();

    Eigen::VectorXd xdot, ydot, zdot;
    Eigen::VectorXd x,    y,    z;

    xdot = rpy_twist_est.col(0);
    ydot = rpy_twist_est.col(1);
    zdot = rpy_twist_est.col(2);

    EXPECT_NEAR( velocities(0) ,xdot.mean(), 0.01 );
    EXPECT_NEAR( velocities(1) ,ydot.mean(), 0.01 );
    EXPECT_NEAR( velocities(2) ,zdot.mean(), 0.01 );

    x = rpy_pose_est.col(0);
    y = rpy_pose_est.col(1);
    z = rpy_pose_est.col(2);

    EXPECT_NEAR( _end_goal_x, x(x.size()-1), 0.01 );
    EXPECT_NEAR( _end_goal_y, y(y.size()-1), 0.01 );
    EXPECT_NEAR( _end_goal_z, z(z.size()-1), 0.01 );

    xdot = rpy_ext_twist_est.col(0);
    ydot = rpy_ext_twist_est.col(1);
    zdot = rpy_ext_twist_est.col(2);

    EXPECT_NEAR( velocities(0) ,xdot.mean(), 0.01 );
    EXPECT_NEAR( velocities(1) ,ydot.mean(), 0.01 );
    EXPECT_NEAR( velocities(2) ,zdot.mean(), 0.01 );

    x = rpy_ext_pose_est.col(0);
    y = rpy_ext_pose_est.col(1);
    z = rpy_ext_pose_est.col(2);

    EXPECT_NEAR( _end_goal_x, x(x.size()-1), 0.01 );
    EXPECT_NEAR( _end_goal_y, y(y.size()-1), 0.01 );
    EXPECT_NEAR( _end_goal_z, z(z.size()-1), 0.01 );
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_manager_test");
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
