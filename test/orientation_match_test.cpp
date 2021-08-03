#include <Eigen/Dense>

#include "target_estimation/utils.hpp"
#include "target_estimation/geometry.hpp"

GradientDescent::Ptr gd;

Eigen::VectorXd gradient(const Eigen::VectorXd& x, const Eigen::Vector3d& omega, const Eigen::Quaterniond& qd, const Eigen::Quaterniond& q)
{
    double t = x(0); // Get the time
    return (qd.coeffs().transpose() * Qtrandot(t,omega) * q.coeffs());
}

int main(int argc, char* argv[])
{

  Eigen::Quaterniond q, qd;
  qd.x() = qd.y() = qd.z() = 0.0;
  qd.w() = 1.0;

  q.x() = 0.0;
  q.y() = 0.0;
  q.z() = 0.0;
  q.w() = 1.0;

  Eigen::Vector3d omega;
  omega << 0.0, 0.0, 10.0;

  gd.reset(new GradientDescent());

  unsigned int n = 300;
  double dt = 0.01;

  Eigen::VectorXd t0(1), topt(1);
  t0(0) = 0.0;

  Eigen::VectorXd topt_out(n);

  double omega_norm = omega.norm();

  double estimated_period = 2*M_PI/omega_norm;

  for (unsigned int i = 0; i< n; i++)
  {

    q = Qtran(dt,omega) * q.coeffs();
    q.normalize();

    topt = gd->solve(std::bind(&gradient,std::placeholders::_1,omega,qd,q),t0);


    if (sgn(topt(0)) < 0.0)
        topt(0) += estimated_period;

    topt_out(i) = topt(0);

  }

  writeTxtFile("topt",topt_out);

  //std::cout <<"topt_out: "<< topt_out << std::endl;
  std::cout <<"Period: " << estimated_period << std::endl;

  return 0;
}

