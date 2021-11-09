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
static double _dt = 0.001;
static unsigned int _n_points = 10000;
static double _end_goal_x = 0.2;
static double _end_goal_y = 0.3;
static double _end_goal_z = 0.4;
static Eigen::Vector3d _omega(3.0, 0.01, 0.1);

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
                          Eigen::MatrixXd& meas_pose, Eigen::MatrixXd& real_pose, Eigen::VectorXd& time)
{
  // List of position measurements x,y,z and quaternion
  // Generate a linear motion on x y z and an angular motion using omega
  time      = Eigen::VectorXd::Zero(n_points);
  meas_pose = Eigen::MatrixXd::Zero(n_points,7);
  real_pose = Eigen::MatrixXd::Zero(n_points,7);
  real_pose.col(0) = Eigen::VectorXd::LinSpaced(n_points, 0.0, end_goal_x); // Move on the x
  real_pose.col(1) = Eigen::VectorXd::LinSpaced(n_points, 0.0, end_goal_y); // Move on the y
  real_pose.col(2) = Eigen::VectorXd::LinSpaced(n_points, 0.0, end_goal_z); // Move on the z
  Eigen::Quaterniond q_meas = Eigen::Quaterniond::Identity();

  double t = 0.0;

  // Update the measurement
  for(unsigned int i=0;i<n_points;i++)
  {
      meas_pose(i,0) = real_pose(i,0) + normal_dist(_generator);
      meas_pose(i,1) = real_pose(i,1) + normal_dist(_generator);
      meas_pose(i,2) = real_pose(i,2) + normal_dist(_generator);

      meas_pose(i,3) = real_pose(i,3) = q_meas.x();
      meas_pose(i,4) = real_pose(i,4) = q_meas.y();
      meas_pose(i,5) = real_pose(i,5) = q_meas.z();
      meas_pose(i,6) = real_pose(i,6) = q_meas.w();

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

void generateEstimation(const unsigned int id, const Eigen::MatrixXd& meas_pose, Eigen::MatrixXd& est_pose, Eigen::MatrixXd& est_twist, Eigen::MatrixXd& sigma_xyz)
{
  // Feed measurements into filter, output estimated states
  est_pose.resize(_n_points,7);
  est_twist.resize(_n_points,6);
  sigma_xyz.resize(_n_points,3);

  Eigen::Vector7d current_meas_pose;
  Eigen::Vector7d current_est_pose;
  Eigen::Vector6d current_est_twist;

  for(unsigned int i = 0; i < _n_points; i++)
  {
      current_meas_pose = meas_pose.row(i);
      _manager.update(id,_dt,current_meas_pose);
      _manager.getTargetPose(id,current_est_pose);
      est_pose.row(i) = current_est_pose;
      _manager.getTargetTwist(id,current_est_twist);
      est_twist.row(i) = current_est_twist;
      sigma_xyz = _manager.getTarget(id)->getEstimator()->getP().topLeftCorner(3,3).diagonal();
  }
}

TEST(test_target, RPY)
{
    Eigen::MatrixXd Q,R,P;
    generateMatrices(Q,R,P);
    Eigen::MatrixXd real_pose, meas_pose;
    Eigen::VectorXd time;
    generateMeasurements(_dt,_n_points,_end_goal_x,_end_goal_y,_end_goal_z,_omega,meas_pose,real_pose,time);

    unsigned int id = 0;
    Eigen::Vector7d p0;
    initPose(p0);
    _manager.init(id,_dt,Q,R,P,p0,0.0,TargetManager::target_t::RPY);

    Eigen::MatrixXd est_pose, est_twist, sigma_xyz;
    generateEstimation(id,meas_pose,est_pose,est_twist,sigma_xyz);

    // Save the data for the plots
    writeTxtFile("time_"+std::to_string(id),time);
    writeTxtFile("real_pose_"+std::to_string(id),real_pose);
    writeTxtFile("meas_pose_"+std::to_string(id),meas_pose);
    writeTxtFile("est_pose_"+std::to_string(id),est_pose);
    writeTxtFile("est_twist_"+std::to_string(id),est_twist);

    auto velocities = calculateVelocities();

    Eigen::VectorXd xdot, ydot, zdot;
    Eigen::VectorXd omegax, omegay, omegaz;
    Eigen::VectorXd x,    y,    z;

    x = est_pose.col(0);
    y = est_pose.col(1);
    z = est_pose.col(2);

    EXPECT_NEAR( _end_goal_x, x(x.size()-1), 0.01 );
    EXPECT_NEAR( _end_goal_y, y(y.size()-1), 0.01 );
    EXPECT_NEAR( _end_goal_z, z(z.size()-1), 0.01 );

    xdot   = est_twist.col(0);
    ydot   = est_twist.col(1);
    zdot   = est_twist.col(2);
    omegax = est_twist.col(3);
    omegay = est_twist.col(4);
    omegaz = est_twist.col(5);

    EXPECT_NEAR( velocities(0) ,xdot.mean(), 0.01 );
    EXPECT_NEAR( velocities(1) ,ydot.mean(), 0.01 );
    EXPECT_NEAR( velocities(2) ,zdot.mean(), 0.01 );
    // Note: for RPY target does not make sense to check if the omegas
    // are correct
    //EXPECT_NEAR( _omega(0) ,omegax.mean(), 0.01 );
    //EXPECT_NEAR( _omega(1) ,omegay.mean(), 0.01 );
    //EXPECT_NEAR( _omega(2) ,omegaz.mean(), 0.01 );
}

TEST(test_target, RPYExt)
{
    Eigen::MatrixXd Q,R,P;
    generateMatrices(Q,R,P);
    Eigen::MatrixXd real_pose, meas_pose;
    Eigen::VectorXd time;
    generateMeasurements(_dt,_n_points,_end_goal_x,_end_goal_y,_end_goal_z,_omega,meas_pose,real_pose,time);

    unsigned int id = 1;
    Eigen::Vector7d p0;
    initPose(p0);
    _manager.init(id,_dt,Q,R,P,p0,0.0,TargetManager::target_t::RPY_EXT);

    Eigen::MatrixXd est_pose, est_twist, sigma_xyz;
    generateEstimation(id,meas_pose,est_pose,est_twist,sigma_xyz);

    // Save the data for the plots
    writeTxtFile("time_"+std::to_string(id),time);
    writeTxtFile("real_pose_"+std::to_string(id),real_pose);
    writeTxtFile("meas_pose_"+std::to_string(id),meas_pose);
    writeTxtFile("est_pose_"+std::to_string(id),est_pose);
    writeTxtFile("est_twist_"+std::to_string(id),est_twist);

    auto velocities = calculateVelocities();

    Eigen::VectorXd xdot, ydot, zdot;
    Eigen::VectorXd omegax, omegay, omegaz;
    Eigen::VectorXd x,    y,    z;

    x = est_pose.col(0);
    y = est_pose.col(1);
    z = est_pose.col(2);

    EXPECT_NEAR( _end_goal_x, x(x.size()-1), 0.01 );
    EXPECT_NEAR( _end_goal_y, y(y.size()-1), 0.01 );
    EXPECT_NEAR( _end_goal_z, z(z.size()-1), 0.01 );

    xdot   = est_twist.col(0);
    ydot   = est_twist.col(1);
    zdot   = est_twist.col(2);
    omegax = est_twist.col(3);
    omegay = est_twist.col(4);
    omegaz = est_twist.col(5);

    EXPECT_NEAR( velocities(0) ,xdot.mean(), 0.01 );
    EXPECT_NEAR( velocities(1) ,ydot.mean(), 0.01 );
    EXPECT_NEAR( velocities(2) ,zdot.mean(), 0.01 );
    EXPECT_NEAR( _omega(0)     ,omegax.mean(), 0.05 );
    EXPECT_NEAR( _omega(1)     ,omegay.mean(), 0.05 );
    EXPECT_NEAR( _omega(2)     ,omegaz.mean(), 0.05 );
    EXPECT_NEAR( _omega(0), omegax(omegax.size()-1), 0.01 );
    EXPECT_NEAR( _omega(1), omegay(omegay.size()-1), 0.01 );
    EXPECT_NEAR( _omega(2), omegaz(omegaz.size()-1), 0.01 );
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_manager_test");
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
