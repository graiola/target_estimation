#include <vector>
#include <random>
#include <Eigen/Dense>

#include "target_estimation/target_manager.hpp"

static TargetManager manager_;
// Simulated measurement noise
const double pObserved_stddev = 0.001; // 1mm
const double pObserved_mean = 0.0;
// Statistical generators
static std::default_random_engine generator;
static std::normal_distribution<double> normal_dist(pObserved_mean, pObserved_stddev);



int main(int argc, char* argv[]) {

  Eigen::MatrixXd Q_rpy_(12,12), R_rpy_(6,6), P_rpy_(12,12);

  Q_rpy_ << 2.5000e-17,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   5.0000e-14,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,
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

  R_rpy_ << 1.0000e-04,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   1.0000e-04,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   1.0000e-04,   0.0000e+00,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-04,   0.0000e+00,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-04,   0.0000e+00,
            0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   0.0000e+00,   1.0000e-04;

  P_rpy_ << 1.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,   0.00000,
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


    // List of position measurements x,y,z and quaternion
    // Generate a linear motion on x and an angular motion on z
    // With dt = 0.001 and 10000 samples we have:
    // estimated dot_x: 0.2
    unsigned int n_points = 10000;
    Eigen::VectorXd time(n_points);
    Eigen::MatrixXd pose = Eigen::MatrixXd::Zero(n_points,7);
    pose.col(0) = Eigen::VectorXd::LinSpaced(n_points, 0.0, 2.0); // Move on the x
    Eigen::MatrixXd pose_meas(n_points,7);
    Eigen::Quaterniond q_meas = Eigen::Quaterniond::Identity();
    double dt = 0.001;
    Eigen::Vector3d omega;
    omega << 3.0, 0.01, 0.1;
    double t = 0.0;

    for(unsigned int i=0;i<n_points;i++)
    {
        pose_meas(i,0) = pose(i,0) + normal_dist(generator);
        pose_meas(i,1) = pose(i,1) + normal_dist(generator);
        pose_meas(i,2) = pose(i,2) + normal_dist(generator);

        pose_meas(i,3) = pose(i,3) = q_meas.x();
        pose_meas(i,4) = pose(i,4) = q_meas.y();
        pose_meas(i,5) = pose(i,5) = q_meas.z();
        pose_meas(i,6) = pose(i,6) = q_meas.w();

        // Update the measurement
        q_meas.coeffs() = Qtran(dt,omega) * q_meas.coeffs();
        q_meas.normalize();
    }

    Eigen::Vector7d p0;
    initPose(p0);
    manager_.init(0,dt,Q_rpy_,R_rpy_,P_rpy_,p0,t,TargetManager::target_t::RPY);
    manager_.init(1,dt,Q_rpy_,R_rpy_,P_rpy_,p0,t,TargetManager::target_t::RPY_EXT);


    // Feed measurements into filter, output estimated states
    Eigen::MatrixXd rpy_pose_est(n_points,7);
    Eigen::MatrixXd rpy_twist_est(n_points,6);
    Eigen::MatrixXd rpy_ext_pose_est(n_points,7);
    Eigen::MatrixXd rpy_ext_twist_est(n_points,6);
    Eigen::MatrixXd quat_pose_est(n_points,7);
    Eigen::MatrixXd quat_twist_est(n_points,6);
    Eigen::Vector7d current_meas;
    Eigen::Vector7d current_pose_est;
    Eigen::Vector6d current_twist_est;

    for(unsigned int i = 0; i < n_points; i++) {

        current_meas = pose_meas.row(i);

        manager_.update(0,dt,current_meas);
        manager_.update(1,dt,current_meas);

        manager_.getTargetPose(0,current_pose_est);
        rpy_pose_est.row(i) = current_pose_est;
        manager_.getTargetTwist(0,current_twist_est);
        rpy_twist_est.row(i) = current_twist_est;

        manager_.getTargetPose(1,current_pose_est);
        rpy_ext_pose_est.row(i) = current_pose_est;
        manager_.getTargetTwist(1,current_twist_est);
        rpy_ext_twist_est.row(i) = current_twist_est;

        time(i) = t += dt;
    }

    writeTxtFile("time",time);
    writeTxtFile("pose",pose);
    writeTxtFile("meas_pose",pose_meas);
    writeTxtFile("rpy_est_pose",rpy_pose_est);
    writeTxtFile("rpy_est_twist",rpy_twist_est);
    writeTxtFile("rpy_ext_est_pose",rpy_ext_pose_est);
    writeTxtFile("rpy_ext_est_twist",rpy_ext_twist_est);


    return 0;
}
