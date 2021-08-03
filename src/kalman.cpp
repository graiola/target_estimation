/**
*/

#include <iostream>
#include <stdexcept>

#include "target_estimation/kalman.hpp"
#include "target_estimation/geometry.hpp"

// ----------------------------
// KalmanFilterInterface
// ----------------------------

KalmanFilterInterface::KalmanFilterInterface() {}

void KalmanFilterInterface::init(const Eigen::VectorXd& x0)
{
  x_hat_ = x0;
  P_ = P0_;
  initialized_ = true;
}

void KalmanFilterInterface::init()
{
  x_hat_.setZero();
  P_ = P0_;
  initialized_ = true;
}

void KalmanFilterInterface::update(const Eigen::VectorXd& y)
{

  if(!initialized_)
    throw std::runtime_error("Filter is not initialized!");

  // Predict
  predict();
  // Estimate
  estimate(y);
  // Output
  x_hat_ = x_hat_new_; // save into old value
}

void KalmanFilterInterface::update()
{

  if(!initialized_)
    throw std::runtime_error("Filter is not initialized!");

  // Predict
  predict();
  // Output
  x_hat_ = x_hat_new_; // save into old value
}

// ----------------------------
// LinearKalmanFilter
// ----------------------------

LinearKalmanFilter::LinearKalmanFilter() {}

LinearKalmanFilter::LinearKalmanFilter(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
{
  A_ = A;
  C_ = C;
  Q_ = Q;
  R_ = R;
  P0_ = P;
  m_ = C.rows();
  n_ = A.rows();
  initialized_ = false;
  I_.resize(n_, n_);
  K_.resize(n_,m_);
  x_hat_.resize(n_);
  x_hat_new_.resize(n_);
  I_.setIdentity();
}

void LinearKalmanFilter::predict()
{
  x_hat_new_ = A_ * x_hat_; // update the prior
  P_ = A_*P_*A_.transpose() + Q_; // prior covariance
}

void LinearKalmanFilter::estimate(const Eigen::VectorXd& y)
{
  K_ = P_*C_.transpose()*(C_*P_*C_.transpose() + R_).inverse(); // update the kalman gain
  x_hat_new_ += K_ * (y - C_*x_hat_new_); // update the posterior
  P_ = (I_ - K_*C_)*P_; // update the posterior covariance
}

void LinearKalmanFilter::update(const Eigen::MatrixXd& A)
{
  A_ = A;
  KalmanFilterInterface::update();
}

void LinearKalmanFilter::update(const Eigen::VectorXd& y, const Eigen::MatrixXd& A)
{
  A_ = A;
  KalmanFilterInterface::update(y);
}

// ----------------------------
// ExtendedKalmanFilter
// ----------------------------

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

ExtendedKalmanFilter::ExtendedKalmanFilter(
    nonliner_function_t f,
    nonliner_function_t h,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : LinearKalmanFilter (A,C,Q,R,P)
{
  f_ = f;
  h_ = h;
}

void ExtendedKalmanFilter::predict()
{
  x_hat_new_ = f_(x_hat_); // update the prior
  P_ = A_*P_*A_.transpose() + Q_; // prior covariance
}

void ExtendedKalmanFilter::estimate(const Eigen::VectorXd& y)
{
  K_ = P_*C_.transpose()*(C_*P_*C_.transpose() + R_).inverse(); // update the kalman gain
  x_hat_new_ += K_ * (y - h_(x_hat_new_)); // update the posterior
  P_ = (I_ - K_*C_)*P_; // update the posterior covariance
}

void ExtendedKalmanFilter::update(nonliner_function_t f, const Eigen::MatrixXd& A)
{
  f_ = f;
  LinearKalmanFilter::update(A);
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd& y, nonliner_function_t f, const Eigen::MatrixXd& A)
{
  f_ = f;
  LinearKalmanFilter::update(y,A);
}

void ExtendedKalmanFilter::update()
{
  KalmanFilterInterface::update();
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd& y)
{
  KalmanFilterInterface::update(y);
}
