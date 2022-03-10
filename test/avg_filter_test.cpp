#include <random>
#include <gtest/gtest.h>
#include "target_estimation/utils.hpp"

// Simulated noise
const double gt_stddev = 1;
const double gt_var    = gt_stddev*gt_stddev;
const double gt_mean = 5.0;
// Statistical generators
static std::default_random_engine generator;
static std::normal_distribution<double> normal_dist(gt_mean,gt_stddev);

TEST(test_filter, AvgFilter )
{
  AvgFilter filter(1000);

  unsigned int n_meas = 10000;
  Eigen::VectorXd mean(n_meas);
  double meas;
  for(unsigned int i=0;i<n_meas;i++)
  {
      meas = normal_dist(generator);
      mean(i) = filter.update(meas);
  }
  EXPECT_NEAR( gt_mean ,mean(n_meas-1), 0.1 );
}

TEST(test_filter, MovingAvgFilter )
{
  MovingAvgFilter filter(1000);

  unsigned int n_meas = 10000;
  Eigen::VectorXd mean(n_meas);
  Eigen::VectorXd var(n_meas);
  double meas;
  for(unsigned int i=0;i<n_meas;i++)
  {
      meas = normal_dist(generator);
      mean(i) = filter.update(meas);
      var(i) = filter.getVariance();
  }
  EXPECT_NEAR( gt_mean ,mean(n_meas-1), 0.1 );
  EXPECT_NEAR( gt_var , var(n_meas-1), 0.1 );
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
