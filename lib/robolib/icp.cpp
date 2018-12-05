#include "icp.h"

#include <iostream> // TODO remove
#include <Eigen/Core>
#include <Eigen/SVD>
#include <math.h>

namespace robo
{
void icpMatch(Eigen::MatrixXd &points1, Eigen::MatrixXd &points2,
              Eigen::Matrix2d &R, Eigen::Vector2d &t)
{
  // ICP algorithm adapted from "Introduction to Mobile Robotics: Iterative
  // Closest Point Algorithm
  // https://cs.gmu.edu/~kosecka/cs685/cs685-icp.pdf
  // Also from the python implementation publically available here
  // https://github.com/AtsushiSakai/PythonRobotics/blob/d7fe083b1c4bfa28ec0d4473d04ed3ac8a78059f/SLAM/iterative_closest_point/iterative_closest_point.py
  R.setIdentity();
  t.setZero();
}

void svdMotionEstimation(Eigen::MatrixXd &prev_points,
                         Eigen::MatrixXd &curr_points, Eigen::Matrix2d &R,
                         Eigen::Vector2d &t)
{
  Eigen::Vector2d prev_mean{ prev_points.rowwise().mean() };
  Eigen::Vector2d curr_mean{ curr_points.rowwise().mean() };

  Eigen::MatrixXd prev_points_shifted{ prev_points.colwise() - prev_mean };
  Eigen::MatrixXd curr_points_shifted{ curr_points.colwise() - curr_mean };

  Eigen::MatrixXd W{ curr_points_shifted * prev_points_shifted.transpose() };
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  R = (svd.matrixU() * svd.matrixV().transpose()).transpose();

  t = prev_mean - (R * curr_mean);
}

void angleTo2DRotationMatrix(double angle, Eigen::Matrix2d& rot_mat)
{
  rot_mat(0, 0) = cos(angle);
  rot_mat(0, 1) = -sin(angle);
  rot_mat(1, 0) = sin(angle);
  rot_mat(1, 1) = cos(angle);
}
}
