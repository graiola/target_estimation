#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <unsupported/Eigen/Polynomials>
#include <cmath>
#include <iostream>
#include <Eigen/src/Core/Matrix.h>
/* Solve n-th order polynomial

    Pass in the coefficients with the zero-th order term first in the vector
    i.e. for a0+a1*x+a2*x^2=0 pass in [a0 a1 a2]
*/

class Solver
{

public:

  Solver(){}

  double lowestRealRoot(const Eigen::VectorXd &coeffs);

private:

  Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;

};


#endif
