#include <random>
#include <gtest/gtest.h>
#include "target_estimation/utils.hpp"

// Simulated noise
const double stddev = 1;
const double mean = 5.0;
// Statistical generators
std::default_random_engine generator;
std::normal_distribution<double> normal_dist(mean,stddev);

TEST(test_filter, AvgFilter )
{
  AvgFilter filter(1000);

  unsigned int n_meas = 10000;
  Eigen::VectorXd res(n_meas);
  double meas;
  for(unsigned int i=0;i<n_meas;i++)
  {
      meas = normal_dist(generator);
      res(i) = filter.update(meas);
  }
  EXPECT_NEAR( mean ,res(n_meas-1), 0.1 );
}

TEST(test_filter, MovingAvgFilter )
{
  MovingAvgFilter filter(1000);

  unsigned int n_meas = 10000;
  Eigen::VectorXd res(n_meas);
  double meas;
  for(unsigned int i=0;i<n_meas;i++)
  {
      meas = normal_dist(generator);
      res(i) = filter.update(meas);
  }
  EXPECT_NEAR( mean ,res(n_meas-1), 0.1 );
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
