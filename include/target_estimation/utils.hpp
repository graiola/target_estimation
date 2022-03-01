/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <Eigen/Core>

// Generic helpers
#define POSE_pos(x)                   x.segment(0,3)
#define POSE_quat(x)                  x.segment(3,4)
#define POSE_rpy(x)                   x.segment(3,3)
#define POSE_x(x)                     x(0)
#define POSE_y(x)                     x(1)
#define POSE_z(x)                     x(2)
#define POSE_qx(x)                    x(3)
#define POSE_qy(x)                    x(4)
#define POSE_qz(x)                    x(5)
#define POSE_qw(x)                    x(6)
#define POSE_roll(x)                  x(3)
#define POSE_pitch(x)                 x(4)
#define POSE_yaw(x)                   x(5)
#define TWIST_linear(x)               x.segment(0,3)
#define TWIST_angular(x)              x.segment(3,3)
#define ACCELERATION_linear(x)        x.segment(0,3)
#define ACCELERATION_angular(x)       x.segment(3,3)

#define GRAVITY                9.81

namespace Eigen
{
    typedef Matrix<double,6,1> Vector6d;
    typedef Matrix<double,7,1> Vector7d;
    typedef Matrix<double,6,6> Matrix6d;
    typedef Matrix<double,4,3> Matrix4x3d;
    typedef Matrix<double,3,6> Matrix3x6d;
}

inline double toSec(uint32_t sec, uint32_t nsec)
{
  return static_cast<double>(sec) + 1e-9*static_cast<double>(nsec);
}

inline void initPose(Eigen::Vector7d& pose)
{
    pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
}

inline void initPose(Eigen::Vector6d& pose)
{
    pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

inline void writeTxtFile(const std::string filename, Eigen::VectorXd& values) {
    std::ofstream myfile (filename.c_str());
    std::size_t row = 0;
    std::size_t nb_rows = values.size();
    if (myfile.is_open())
    {
        while(row < nb_rows) {
        myfile << values(row) << "\n";
            row++;
        }
    std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows ] "<<std::endl;
    }
    else{
     std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
    }
    myfile.close();
}

inline void writeTxtFile(const std::string filename, Eigen::MatrixXd& values) {
    std::ofstream myfile (filename.c_str());
    std::size_t row = 0;
    std::size_t nb_rows = values.rows();
    std::size_t col = 0;
    std::size_t nb_cols = values.cols();

    if (myfile.is_open())
    {
        while(row < nb_rows) {
            while(col < nb_cols) {
                myfile << values(row,col) << " ";
                col++;
            }
            col = 0;
            row++;
            myfile << "\n";
        }
    std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows ]["<<nb_cols<<" cols ] "<<std::endl;
    }
    else{
     std::cerr << "Unable to open file : ["<<filename<<"]"<<std::endl;
    }
    myfile.close();
}

class GradientDescent
{

public:
    typedef std::function<Eigen::VectorXd (const Eigen::VectorXd&)> gradient_t;

    typedef std::shared_ptr<GradientDescent> Ptr;

    GradientDescent(double alpha = 0.05, unsigned int max_num_iters = 200, double J_th = 0.0001)
    {
        assert(std::abs(alpha)>=0.0001);
        assert(max_num_iters>=1);
        assert(J_th>0.0);
        alpha_ = alpha;
        max_num_iters_ = max_num_iters;
        J_th_ = J_th;
    }

    inline Eigen::VectorXd solve(gradient_t grad, const Eigen::VectorXd& x0)
    {
        J_ = grad(x0);
        x_ = x0;
        num_iters_ = 0;
        cost_ = 0.0;

        do
        {
            // Gradient descent
            x_ = x_ - alpha_ * J_;

            // Update for the next iteration
            J_ = grad(x_);
            J_norm_ = J_.norm();
            num_iters_ ++;

            // Save the cumulative cost of J at every iteration
            cost_ += J_norm_;

        }
        while(J_norm_>J_th_ && num_iters_ < max_num_iters_);

        return x_;
    }

    inline double getCost() {return cost_;}

private:

    double alpha_;
    unsigned int max_num_iters_;
    Eigen::VectorXd x_;
    Eigen::VectorXd J_;
    unsigned int num_iters_;
    double cost_;
    double J_norm_;
    double J_th_;

};

class AvgFilter
{
public:
    typedef std::shared_ptr<AvgFilter> Ptr;

    AvgFilter(unsigned int n)
    {
       n_ = n;
       avg_ = 0.0;
    }

    double update(double value)
    {
        avg_ = (avg_ * (n_-1) + value) / n_;
        return avg_;
    }


private:

    unsigned int n_;
    double avg_;

};

class MovingAvgFilter
{

public:
  typedef std::shared_ptr<MovingAvgFilter> Ptr;

  MovingAvgFilter(unsigned int n)
  {
    window_.resize(n);
    for(unsigned int i=0; i<n; i++)
      window_[i] = 0.0;
    window_idx_ = 0;
    sum_ = 0.0;
    filter_complete_ = false;
  }

  double update(double value)
  {
      unsigned int n = window_.size();
      double res = 0.0;

      sum_ -= window_[window_idx_];
      sum_ += value;
      window_[window_idx_] = value;

      if (!filter_complete_ && window_idx_ == n - 1) {
          filter_complete_ = true;
      }
      unsigned int num = n;
      if (!filter_complete_) {
        num = window_idx_ + 1;
      }

      res = sum_ / num;

      window_idx_ = (window_idx_+1) % n;

      double variance_sum = 0.0;
      for(double val : window_)
      {
        variance_sum += std::pow((val - res), 2);
      }
      variance_ = variance_sum / num;

      return res;
  }

  double getVariance()
  {
    return variance_;
  }

private:

  double sum_;
  double variance_;
  std::vector<double> window_;
  unsigned int window_idx_;
  bool filter_complete_;
};

/**
 * @brief Split a string into a list of strings given a delimiter
 * @param s: input string to be splitted
 * @param delimiter: delimiter for splitting; default value = "_".
 * @return vector of strings, which is the copy of input string in case the delimiter is not found
 */
inline std::vector<std::string> splitString(const std::string& s, std::string delimiter = "_")
{
  size_t pos_start = 0, pos_end, delim_len = delimiter.length();
  std::string token;
  std::vector<std::string> res;

  pos_end = s.find (delimiter, pos_start);

  while ( pos_end != std::string::npos)
  {
    token = s.substr (pos_start, pos_end - pos_start);

    pos_start = pos_end + delim_len;

    res.push_back (token);

    pos_end = s.find (delimiter, pos_start);
  }

  res.push_back (s.substr (pos_start));
  return res;
}

/**
 * @brief get the numeric id from a string
 * @param s: input string with format 'xxx_id'
 * @param id: output numeric id
 * @return true if id extraction is successful
 */
inline bool getId(const std::string& s, unsigned int& id)
{
  auto strings = splitString(s);
  if(strings.size() == 2) // 'xxx_id'
  {
    id = std::stoi(strings[1]);
    return true;
  }
  else
    return false;

}


#endif
