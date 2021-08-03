#include "target_estimation/solver.hpp"

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
