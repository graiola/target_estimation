#include "target_estimation/utils.hpp"
#include <random>


// Simulated noise
const double stddev = 1;
const double mean = 5.0;
// Statistical generators
std::default_random_engine generator;
std::normal_distribution<double> normal_dist(mean,stddev);

int main(int argc, char* argv[]) {

  MovingAvgFilter filter(500);

  unsigned int n_meas = 1000;
  Eigen::VectorXd res(n_meas);
  double meas;
  for(unsigned int i=0;i<n_meas;i++)
  {
      meas = normal_dist(generator);
      res(i) = filter.update(meas);
  }

  writeTxtFile("filter_test_res",res);

  return 0;
}
